"""Apply a documented, idempotent appearance pass to the generated X950 scene.

Run inside Blender after adding all generated visual meshes. Does not save the
blend, export URDF, modify the source scene, or assign physical mass properties.
Edit config/materials.json to refine appearances; object['material_key_override']
can opt a generated visual into a specific key from that catalog.
"""
import fnmatch
import json
from pathlib import Path

import bpy


OWNER = "x950_description.apply_materials.v1"
PREFIX = "X950_finish_"
SCENE_NAME = "X950_ROS_LinkForge"


def resolve_material(name, catalog):
    for rule in catalog["rules"]:
        if any(fnmatch.fnmatchcase(name, pattern) for pattern in rule["patterns"]):
            return rule["material"]
    return catalog["fallback_material"]


def create_material(key, specification):
    name = PREFIX + key
    material = bpy.data.materials.get(name)
    if material is not None:
        assert material.get("appearance_owner") == OWNER, (
            "Refusing to overwrite an unmanaged material", name)
    else:
        material = bpy.data.materials.new(name)
    rgba = tuple(specification["rgba"])
    material.diffuse_color = rgba  # LinkForge / URDF / RViz and solid viewport.
    material.use_nodes = True
    nodes = material.node_tree.nodes
    nodes.clear()
    output = nodes.new("ShaderNodeOutputMaterial")
    output.location = (300, 0)
    shader = nodes.new("ShaderNodeBsdfPrincipled")
    shader.location = (0, 0)
    shader.label = specification["label"]
    shader.inputs["Base Color"].default_value = rgba
    shader.inputs["Metallic"].default_value = specification["metallic"]
    shader.inputs["Roughness"].default_value = specification["roughness"]
    shader.inputs["Alpha"].default_value = rgba[3]
    material.node_tree.links.new(shader.outputs["BSDF"], output.inputs["Surface"])
    material["appearance_owner"] = OWNER
    material["material_key"] = key
    material["display_label"] = specification["label"]
    for field in ("engineering_material", "material_status", "appearance_status"):
        material[field] = specification[field]
    material["material_properties_scope"] = "Visual shader only; no density, mass, inertia or optical properties."
    return material


def main():
    scene = bpy.context.scene
    assert scene.name == SCENE_NAME, "Select the generated X950_ROS_LinkForge scene."
    assert bpy.data.filepath, "The source or generated Blender file must have a path."
    out = Path(bpy.data.filepath).resolve().parent
    if out.name != "x950_description":
        out = out / "x950_description"
    path = out / "config" / "materials.json"
    catalog = json.loads(path.read_text())
    assert catalog["schema_version"] == 1 and catalog["scene"] == scene.name
    specifications = catalog["materials"]
    for key, specification in specifications.items():
        rgba = specification["rgba"]
        assert len(rgba) == 4 and all(0.0 <= value <= 1.0 for value in rgba), key
        assert rgba[3] == 1.0, "This physical-visual catalog is intentionally opaque."
        assert 0.0 <= specification["metallic"] <= 1.0
        assert 0.0 <= specification["roughness"] <= 1.0
    for rule in catalog["rules"]:
        assert rule["material"] in specifications
    assert catalog["fallback_material"] in specifications

    visuals = sorted((obj for obj in scene.objects
                      if obj.type == "MESH" and "_visual" in obj.name), key=lambda obj: obj.name)
    assert visuals, "No generated visual meshes found."
    # Validate isolation before changing anything. The original aircraft scene
    # must never be changed through a shared Object or managed Material block.
    for obj in visuals:
        assert all(other == scene for other in obj.users_scene), (
            "Generated visual is shared with another scene", obj.name)
    visual_set = set(visuals)
    used_keys = {obj.get("material_key_override") or resolve_material(obj.name, catalog)
                 for obj in visuals}
    assert used_keys <= specifications.keys(), "Unknown material_key_override."
    for obj in bpy.data.objects:
        if obj in visual_set or obj.type != "MESH":
            continue
        for slot in obj.material_slots:
            if slot.material and slot.material.name in {PREFIX + key for key in used_keys}:
                raise AssertionError(("Managed finish is shared outside generated visuals", obj.name))

    materials = {key: create_material(key, specifications[key]) for key in sorted(used_keys)}
    assignments = []
    for obj in visuals:
        key = obj.get("material_key_override") or resolve_material(obj.name, catalog)
        specification = specifications[key]
        matrix_before = tuple(value for row in obj.matrix_world for value in row)
        geometry_before = (len(obj.data.vertices), len(obj.data.edges), len(obj.data.polygons))
        # Mesh material slots belong to the Mesh datablock: detach shared data
        # before replacing slots, so the source scene's data cannot be changed.
        if obj.data.users > 1:
            obj.data = obj.data.copy()
        obj.data.materials.clear()
        obj.data.materials.append(materials[key])
        for polygon in obj.data.polygons:
            polygon.material_index = 0
        obj.color = tuple(specification["rgba"])
        obj["appearance_owner"] = OWNER
        obj["material_key"] = key
        for field in ("engineering_material", "material_status", "appearance_status"):
            obj[field] = specification[field]
        obj["material_properties_scope"] = "Visual appearance only; physical properties unchanged."
        assert tuple(value for row in obj.matrix_world for value in row) == matrix_before
        assert (len(obj.data.vertices), len(obj.data.edges), len(obj.data.polygons)) == geometry_before
        assignments.append({"object": obj.name, "material": materials[key].name,
                            "material_key": key, "engineering_material": specification["engineering_material"],
                            "material_status": specification["material_status"],
                            "appearance_status": specification["appearance_status"]})

    # Color-based viewport modes now agree with the catalog. Coverage guides,
    # lights, camera, frame markers and source-scene materials are untouched.
    scene.display.shading.color_type = "MATERIAL"
    for screen in bpy.data.screens:
        for area in screen.areas:
            if area.type == "VIEW_3D" and screen == bpy.context.screen:
                area.spaces.active.shading.color_type = "MATERIAL"
    scene["materials_catalog"] = "config/materials.json"
    scene["materials_scope"] = catalog["scope"]
    scene["materials_unclassified"] = json.dumps([
        row["object"] for row in assignments if row["material_key"] == catalog["fallback_material"]])
    bpy.context.view_layer.update()
    return {"scene": scene.name, "visuals_assigned": len(assignments),
            "material_count": len(materials), "assignments": assignments,
            "unclassified_visuals": json.loads(scene["materials_unclassified"]),
            "catalog": str(path), "geometry_and_transforms_unchanged": True,
            "physical_properties_assigned": False}


if __name__ == "__main__":
    result = main()
