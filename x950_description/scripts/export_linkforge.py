"""Run inside Blender to export the X950 scene through LinkForge.

Usage from Blender's Python console or the Blender MCP extension::

    exec(compile(open('/path/to/x950_description/scripts/export_linkforge.py').read(),
                 '/path/to/x950_description/scripts/export_linkforge.py', 'exec'))

Set the global X950_PACKAGE_ROOT to override the output package directory.
The scene remains editable with LinkForge. Its stock Blender Export button adds
mass/inertia defaults to every link; use THIS exporter for the visualization/TF
model until measured mass properties are available. No placeholder physics or
simulator sensor configuration is written to the robot description.
"""

from __future__ import annotations

import importlib
import json
import math
import struct
import sys
import xml.etree.ElementTree as ET
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path

import bpy


XACRO_NS = "http://www.ros.org/wiki/xacro"
PACKAGE_NAME = "x950_description"
ET.register_namespace("xacro", XACRO_NS)


def _package_root():
    override = globals().get("X950_PACKAGE_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    # exec(open(...).read()) often inherits an unrelated __file__.
    source = globals().get("__file__")
    if source and Path(source).name == "export_linkforge.py":
        return Path(source).resolve().parents[1]
    if bpy.data.filepath:
        blend_path = Path(bpy.data.filepath).resolve()
        for parent in blend_path.parents:
            if parent.name == PACKAGE_NAME:
                return parent
            candidate = parent / PACKAGE_NAME
            if (candidate / "scripts" / "export_linkforge.py").is_file():
                return candidate
    return Path(__file__).resolve().parents[1] if "__file__" in globals() else Path.cwd() / PACKAGE_NAME


def _linkforge_prefix():
    for name in bpy.context.preferences.addons.keys():
        if name == "linkforge" or name.endswith(".linkforge"):
            return name
    for name in sys.modules:
        if name.endswith(".linkforge"):
            return name
    raise RuntimeError("Enable the LinkForge extension in this Blender session first")


def _issue_dict(issue):
    return {
        "severity": issue.severity.value,
        "title": issue.title,
        "message": issue.message,
        "affected_objects": list(issue.affected_objects),
        "suggestion": issue.suggestion,
        "code": issue.code.name if issue.code else None,
    }


def _require_unit_scale(obj):
    scale = tuple(obj.matrix_world.to_scale())
    if any(not math.isfinite(v) or abs(v - 1.0) > 1e-5 for v in scale):
        raise ValueError(f"Apply scale before export: {obj.name}: {scale}")
    if obj.matrix_world.to_3x3().determinant() <= 0:
        raise ValueError(f"Apply mirrored transforms before export: {obj.name}")


def _xml_signature(element):
    # Material order and insignificant whitespace are not URDF semantics.
    return (
        element.tag,
        tuple(sorted(element.attrib.items())),
        (element.text or "").strip(),
        tuple(sorted((_xml_signature(child) for child in element), key=repr)),
    )


def _add_calibration_arguments(path, attachment_children):
    tree = ET.parse(path)
    root = tree.getroot()
    args = {}
    parameterized = []
    insertion_index = 0
    for joint in root.findall("joint"):
        child = joint.find("child").get("link")
        if child not in attachment_children:
            continue
        assert joint.get("type") == "fixed", joint.attrib
        origin = joint.find("origin")
        if origin is None:
            origin = ET.SubElement(joint, "origin", xyz="0 0 0", rpy="0 0 0")
        stem = child.removesuffix("_link")
        entry = {"joint": joint.get("name"), "child": child}
        for attribute in ("xyz", "rpy"):
            name = f"{stem}_{attribute}"
            default = origin.get(attribute, "0 0 0")
            args[name] = default
            root.insert(insertion_index, ET.Element(
                f"{{{XACRO_NS}}}arg", name=name, default=default
            ))
            insertion_index += 1
            origin.set(attribute, f"$(arg {name})")
            entry[attribute] = {"argument": name, "default": default}
        parameterized.append(entry)
    assert len(parameterized) == len(attachment_children), parameterized
    ET.indent(tree, space="  ")
    # LinkForge serialization restores ElementTree's global namespace map.
    # Register immediately before our write so ROS xacro sees literal xacro:arg.
    ET.register_namespace("xacro", XACRO_NS)
    tree.write(path, encoding="utf-8", xml_declaration=True)
    return args, parameterized


def _check_xml(urdf_path, xacro_path, args, expected_links, expected_visuals):
    urdf = ET.parse(urdf_path).getroot()
    xacro = ET.parse(xacro_path).getroot()
    for root in (urdf, xacro):
        assert len(root.findall("link")) == expected_links
        assert len(root.findall("joint")) == expected_links - 1
        assert len(root.findall(".//visual")) == expected_visuals
        assert len(root.findall(".//visual/geometry/mesh")) == expected_visuals
        assert not root.findall(".//inertial")
        assert not root.findall("gazebo")
        assert not root.findall("ros2_control")
        assert all(j.get("type") == "fixed" for j in root.findall("joint"))
        for mesh in root.findall(".//mesh"):
            assert mesh.get("filename").startswith(f"package://{PACKAGE_NAME}/meshes/")
    # This Xacro intentionally contains only arg declarations/substitutions.
    # Resolve that restricted syntax to prove defaults match the plain URDF.
    for arg in list(xacro):
        if arg.tag == f"{{{XACRO_NS}}}arg":
            xacro.remove(arg)
    for element in xacro.iter():
        assert not element.tag.startswith(f"{{{XACRO_NS}}}"), element.tag
        for key, value in list(element.attrib.items()):
            for name, default in args.items():
                value = value.replace(f"$(arg {name})", default)
            assert "$(" not in value and "${" not in value, value
            element.set(key, value)
    assert _xml_signature(urdf) == _xml_signature(xacro), "Xacro defaults differ from URDF"


def _check_native_xacro(prefix, urdf_path, xacro_path, parameterized):
    """Exercise LinkForge's actual resolver, including all calibration overrides.

    This is a native LinkForge parser check, not execution of ROS's xacro tool.
    Keep the independent restricted-substitution check above as a cross-check.
    """
    parser_module = importlib.import_module(
        prefix + ".core.parsers.xacro_parser"
    )
    # LinkForge 1.5.3 caches templates globally by resolved path only, without
    # checking mtime/content. A fresh resolver therefore still sees a previous
    # export at this same path in a long-running Blender session. We have just
    # rewritten the Xacro, so invalidate the cache through its public API.
    # All override checks below then reuse this export's freshly parsed data.
    parser_module.clear_xacro_cache()
    resolver_type = parser_module.XacroResolver
    urdf = ET.parse(urdf_path).getroot()
    resolver = resolver_type(start_dir=xacro_path.parent)
    expanded = ET.fromstring(resolver.resolve_file(xacro_path))
    assert _xml_signature(urdf) == _xml_signature(expanded), (
        "LinkForge native Xacro expansion differs from the plain URDF"
    )
    checked_overrides = []
    for entry in parameterized:
        for attribute in ("xyz", "rpy"):
            argument = entry[attribute]["argument"]
            values = [float(v) for v in entry[attribute]["default"].split()]
            values[0] += 0.001
            override = " ".join(format(v, ".12g") for v in values)
            resolver = resolver_type(start_dir=xacro_path.parent)
            resolver.args[argument] = override
            expanded = ET.fromstring(resolver.resolve_file(xacro_path))
            expected = ET.parse(urdf_path).getroot()
            joint = next(j for j in expected.findall("joint")
                         if j.get("name") == entry["joint"])
            joint.find("origin").set(attribute, override)
            assert _xml_signature(expected) == _xml_signature(expanded), (
                f"Calibration override failed or changed unrelated fields: {argument}"
            )
            checked_overrides.append(argument)
    return {
        "engine": prefix + ".core.parsers.xacro_parser.XacroResolver",
        "structural_template_cache_cleared_before_validation": True,
        "default_expansion_matches_urdf": True,
        "override_arguments_checked": checked_overrides,
        "official_ros_xacro_runtime_tested": False,
    }


def export_x950():
    package_root = _package_root()
    helper = package_root / "scripts/scene_helpers.py"
    namespace = {}
    exec(compile(helper.read_text(), str(helper), "exec"), namespace)
    namespace["refresh_manifest"]()
    mesh_dir = package_root / "meshes"
    urdf_dir = package_root / "urdf"
    docs_dir = package_root / "docs"
    for directory in (mesh_dir, urdf_dir, docs_dir):
        directory.mkdir(parents=True, exist_ok=True)
    report_path = docs_dir / "linkforge_validation.json"
    report = {
        "status": "started",
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "blender_version": bpy.app.version_string,
        "scope": "Visual geometry and fixed TF tree; not a dynamics/simulation model",
        "physics": "Supplied sensor masses stored in config/Blender; URDF inertials omitted pending complete physical properties",
        "excluded_validation_checks": {
            "MassPropertiesCheck": "Physical properties are outside this first-pass model"
        },
    }
    try:
        prefix = _linkforge_prefix()
        adapter = importlib.import_module(prefix + ".adapters.blender_to_core")
        core = importlib.import_module(prefix + ".core")
        checks = importlib.import_module(prefix + ".core.validation.checks")
        geometry = importlib.import_module(prefix + ".core.models.geometry")
        report["linkforge_module"] = prefix
        scene = bpy.context.scene
        links = [o for o in scene.objects if o.linkforge.is_robot_link]
        link_names = {o.linkforge.link_name for o in links}
        assert "base_link" in link_names, link_names
        mounts = {name + "_mount_link" for name in ("avia", "d555", "e1r")}
        assert mounts <= link_names, mounts - link_names
        sensors = {name.removesuffix("_mount_link") + "_link" for name in mounts}
        assert sensors <= link_names, sensors - link_names
        attachment_children = mounts | sensors
        parameterized_children = attachment_children | ({"d555_nominal_camera_link", "battery_support_link", "battery_link", "avionics_carrier_link", "fmu_housing_link", "hadron_mount_link", "hadron_link", "hadron_nominal_thermal_optical_frame", "hadron_nominal_visible_optical_frame", "gnss_main_link", "gnss_aux_link", "gnss_main_mount_link", "gnss_aux_mount_link", "hflow_link", "hflow_nominal_flow_optical_frame", "hflow_nominal_range_frame", "hflow_nominal_frd_frame"} & link_names)
        expected_visuals = sum(
            1 for link in links for child in link.children
            if "_visual" in child.name and child.type == "MESH"
        )
        visual_links = {
            link.linkforge.link_name for link in links
            if any("_visual" in c.name and c.type == "MESH" for c in link.children)
        }
        assert ({"base_link"} | attachment_children) <= visual_links, visual_links
        assert expected_visuals >= 7
        for link in links:
            _require_unit_scale(link)
            # STL carries shape only; URDF carries each visual's display color.
            link.linkforge.use_material = True
            for child in link.children:
                if "_visual" in child.name or "_collision" in child.name:
                    child["source_name"] = child.name
                    _require_unit_scale(child)
                    assert child.type == "MESH", child.name
                    assert child.linkforge_geom.geometry_type == "mesh", child.name
        report["scene_link_count"] = len(links)
        report["scene_visual_mesh_count"] = expected_visuals
        props = scene.linkforge_robot
        props.robot_name = "x950"
        props.mesh_format = "STL"
        props.export_meshes = True
        props.use_ros2_control = False
        # Export only once: LinkForge dry-run and actual mesh centering differ.
        robot, conversion = adapter.scene_to_robot(
            bpy.context, meshes_dir=mesh_dir, dry_run=False, raise_on_error=False
        )
        report["conversion_issues"] = [_issue_dict(i) for i in conversion.issues]
        assert conversion.is_valid, [str(i) for i in conversion.errors]
        assert len(robot.links) == len(links)
        assert sum(len(link.visuals) for link in robot.links) == expected_visuals
        assert len(robot.joints) == len(links) - 1
        # LinkForge's Blender adapter unconditionally constructs inertials.
        # Removing them here prevents its defaults from becoming claimed data.
        exported_meshes = []
        for link in robot.links:
            link.inertial = None
            for attribute in ("visuals", "collisions"):
                new_geometry = []
                for visual in getattr(link, attribute):
                    geom = visual.geometry
                    assert isinstance(geom, geometry.Mesh), (
                        f"Mesh export fell back to a primitive: {link.name} ({attribute})"
                    )
                    path = Path(geom.resource).resolve()
                    assert path.parent == mesh_dir.resolve(), path
                    assert path.suffix == ".stl" and path.is_file(), path
                    # Blender's STL export is binary; empty/partial exports fail here.
                    with path.open("rb") as stream:
                        header = stream.read(84)
                    assert len(header) == 84, path
                    triangle_count = struct.unpack("<I", header[80:84])[0]
                    assert triangle_count > 0 and path.stat().st_size == 84 + 50 * triangle_count, path
                    uri = f"package://{PACKAGE_NAME}/meshes/{path.name}"
                    new_geometry.append(replace(visual, geometry=replace(geom, resource=uri)))
                    exported_meshes.append({
                        "link": link.name, "role": attribute,
                        "uri": uri, "triangles": triangle_count,
                    })
                setattr(link, attribute, tuple(new_geometry))
        assert len({m["uri"] for m in exported_meshes}) == len(exported_meshes)
        robot.sensors = ()
        robot.ros2_controls = ()
        robot.transmissions = ()
        robot.gazebo_elements = ()
        robot.extra_elements = ()
        validator = core.RobotValidator(checks=[
            cls() for cls in core.RobotValidator.DEFAULT_CHECKS
            if cls is not checks.MassPropertiesCheck
        ])
        validation = validator.validate(robot)
        report["validation_issues"] = [_issue_dict(i) for i in validation.issues]
        assert validation.is_valid, [str(i) for i in validation.errors]
        urdf_path = urdf_dir / "x950.urdf"
        xacro_path = urdf_dir / "x950.urdf.xacro"
        core.URDFGenerator(output_path=urdf_path, use_ros2_control=False).write(
            robot, urdf_path, validate=False
        )
        core.XACROGenerator(
            output_path=xacro_path, use_ros2_control=False,
            advanced_mode=True, extract_materials=False, extract_dimensions=False,
            generate_macros=False, split_files=False,
        ).write(robot, xacro_path, validate=False)
        args, parameterized = _add_calibration_arguments(xacro_path, parameterized_children)
        _check_xml(urdf_path, xacro_path, args, len(links), expected_visuals)
        native_xacro_check = _check_native_xacro(prefix, urdf_path, xacro_path, parameterized)
        report.update({
            "status": "passed", "is_valid": True,
            "link_count": len(robot.links), "joint_count": len(robot.joints),
            "visual_mesh_count": expected_visuals,
            "mesh_files": exported_meshes,
            "urdf": str(urdf_path), "xacro": str(xacro_path),
            "calibration_arguments": parameterized,
            "xacro_defaults_match_urdf": True,
            "xacro_default_check": "Independent arg substitution and native LinkForge XacroResolver",
            "native_xacro_validation": native_xacro_check,
            "error_count": conversion.error_count + validation.error_count,
            "warning_count": conversion.warning_count + validation.warning_count,
        })
        return report
    except Exception as exc:
        report.update({"status": "failed", "is_valid": False, "error": repr(exc)})
        raise
    finally:
        report_path.write_text(json.dumps(report, indent=2) + "\n")


EXPORTED_RESULT = export_x950()
result = EXPORTED_RESULT
print(json.dumps(EXPORTED_RESULT, indent=2))
