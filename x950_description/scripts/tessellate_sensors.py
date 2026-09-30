#!/usr/bin/env python3
"""Tessellate the three supplied STEP bodies without changing their CAD frames.

Requires cadquery-ocp / OCP in the active Python environment. No packages are
installed by this script. Output STL coordinates are MILLIMETRES, exactly as in
the STEP assemblies. Apply the documented sensor/aircraft/ROS transforms once
when making the final robot meshes; do not load these source meshes as metres.

Example from an environment with OCP available:
    python tessellate_sensors.py --source-root ../.. --output-root ..
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import struct

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS_Compound


SOURCES = (
    ("avia", "Avia/Livox_Avia_shell_FOV.stp", 1),
    ("d555", "D555/D555_SOLID_SOC.STEP", 14),
    ("e1r", "E1R/E1R 3D数模.STEP", 25),
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def bounds(shape) -> list[list[float]]:
    """Exact B-rep bounds, without inflated tessellation tolerances."""
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    values = box.Get()
    return [[round(float(x), 9) for x in values[:3]],
            [round(float(x), 9) for x in values[3:]]]


def shapes(shape, kind, avoid=None):
    explorer = (TopExp_Explorer(shape, kind) if avoid is None
                else TopExp_Explorer(shape, kind, avoid))
    while explorer.More():
        yield explorer.Current()
        explorer.Next()


def clean_and_inspect_binary_stl(path: Path) -> dict:
    """Drop zero-area facets collapsed by STL float32 precision, then validate."""
    data = path.read_bytes()
    if len(data) < 84:
        raise RuntimeError(f"STL export is too short: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if not count or len(data) != 84 + 50 * count:
        raise RuntimeError(f"Invalid binary STL length/count: {path}")
    lower, upper = [math.inf] * 3, [-math.inf] * 3
    zero_area = 0
    facets = []
    for offset in range(84, len(data), 50):
        xyz = struct.unpack_from("<12f", data, offset)[3:]
        if not all(math.isfinite(value) for value in xyz):
            raise RuntimeError(f"Non-finite STL vertex: {path}")
        a = [xyz[3 + i] - xyz[i] for i in range(3)]
        b = [xyz[6 + i] - xyz[i] for i in range(3)]
        cross = (a[1] * b[2] - a[2] * b[1],
                 a[2] * b[0] - a[0] * b[2],
                 a[0] * b[1] - a[1] * b[0])
        if sum(component * component for component in cross) < 1e-24:
            zero_area += 1
            continue
        facets.append(data[offset:offset + 50])
        for index, value in enumerate(xyz):
            axis = index % 3
            lower[axis] = min(lower[axis], value)
            upper[axis] = max(upper[axis], value)
    if not facets:
        raise RuntimeError(f"STL has no non-degenerate facets: {path}")
    if zero_area:
        data = data[:80] + struct.pack("<I", len(facets)) + b"".join(facets)
        path.write_bytes(data)
    return {
        "triangle_count": len(facets),
        "raw_triangle_count": count,
        "bbox_mm": [lower, upper],
        "discarded_zero_area_triangles": zero_area,
        "zero_area_triangles": 0,
        "byte_count": len(data),
        "sha256": sha256(path),
    }


def convert(name: str, relative_source: str, expected_count: int,
            source_root: Path, output_root: Path, linear: float, angular: float) -> dict:
    source = source_root / relative_source
    reader = STEPControl_Reader()
    if reader.ReadFile(str(source)) != IFSelect_RetDone:
        raise RuntimeError(f"Cannot read STEP: {source}")
    if not reader.TransferRoots():
        raise RuntimeError(f"Cannot transfer STEP assembly: {source}")
    assembly = reader.OneShape()
    inventory, kept = [], []
    for index, solid in enumerate(shapes(assembly, TopAbs_SOLID), 1):
        box = bounds(solid)
        size = [box[1][axis] - box[0][axis] for axis in range(3)]
        # Avia includes a metre-scale FOV cone alongside its ~91mm housing.
        # No physical body in these files has an extent greater than 300mm.
        reason = "Physical sensor geometry"
        excluded = name == "avia" and max(size) > 300.0
        if excluded:
            reason = "Avia FOV volume; not physical geometry"
        if name == "e1r" and max(size) < 1.1:
            center = [(box[0][axis] + box[1][axis]) / 2 for axis in range(3)]
            # These STEP products are explicitly named 光心 (optical centre)
            # and 质心 (centre of mass), and are construction marker solids.
            for label, datum in (("optical centre", (0.01, 0.0, 5.5879)),
                                 ("centre of mass", (0.01, -6.35, 18.881))):
                if all(abs(center[axis] - datum[axis]) < 0.01 for axis in range(3)):
                    excluded = True
                    reason = f"E1R CAD {label} marker; not physical geometry"
        inventory.append({
            "kind": "solid", "index": index, "bbox_mm": box,
            "dimensions_mm": size, "kept": not excluded,
            "reason": reason,
        })
        if not excluded:
            kept.append(solid)

    # STEPControl retains E1R's FOV sheet models. Enumerating only solids above
    # deliberately excludes these open shells; inventory them for traceability.
    for index, shell in enumerate(shapes(assembly, TopAbs_SHELL, TopAbs_SOLID), 1):
        inventory.append({
            "kind": "open_shell", "index": index, "bbox_mm": bounds(shell),
            "kept": False, "reason": "Non-solid sheet/FOV construction geometry",
        })
    for index, face in enumerate(shapes(assembly, TopAbs_FACE, TopAbs_SHELL), 1):
        inventory.append({
            "kind": "standalone_face", "index": index, "bbox_mm": bounds(face),
            "kept": False, "reason": "Non-solid sheet/FOV construction geometry",
        })

    if not kept:
        raise RuntimeError(f"{name}: no physical solids found")
    if name == "avia" and sum(not item["kept"] and item["kind"] == "solid"
                              for item in inventory) != 1:
        raise RuntimeError("Avia: expected exactly one excluded FOV solid")

    compound = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(compound)
    for solid in kept:
        builder.Add(compound, solid)
    physical_bounds = bounds(compound)
    mesher = BRepMesh_IncrementalMesh(compound, linear, False, angular, True)
    if not mesher.IsDone():
        raise RuntimeError(f"{name}: tessellation did not finish")
    output = output_root / "cad_source_meshes" / f"source_{name}.stl"
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    if not writer.Write(compound, str(output)):
        raise RuntimeError(f"{name}: STL export failed")
    stl = clean_and_inspect_binary_stl(output)
    print(f"{name}: {len(kept)} solids, {stl['triangle_count']} triangles, "
          f"bbox_mm={stl['bbox_mm']}", flush=True)
    return {
        "source": relative_source, "source_sha256": sha256(source),
        "output": f"cad_source_meshes/source_{name}.stl", "output_units": "mm",
        "transformation": "None; STEP assembly locations applied by STEPControl",
        "kept_solid_count": len(kept),
        "documented_source_solid_count": expected_count,
        "solid_count_note": (
            "STEP importer split the Avia housing and M12 connector into separate solids"
            if name == "avia" and len(kept) == 2 else
            "25 source solids include 2 CAD datum markers; retained 23 physical solids"
            if name == "e1r" and len(kept) == 23 else
            "Imported count matches source record" if len(kept) == expected_count else
            "Imported count differs from source record; retained all physical solids"),
        "brep_bbox_mm": physical_bounds,
        "inventory": inventory, "stl": stl,
    }


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", type=Path, default=package.parent)
    parser.add_argument("--output-root", type=Path, default=package)
    parser.add_argument("--linear-deflection", type=float, default=0.15,
                        help="Absolute chord deflection in mm (default: 0.15)")
    parser.add_argument("--angular-deflection", type=float, default=0.35,
                        help="Angular deflection in radians (default: 0.35)")
    args = parser.parse_args()
    if args.linear_deflection <= 0 or not 0 < args.angular_deflection < math.pi:
        parser.error("Deflections must be positive; angle must be below pi")
    report = {
        "schema_version": 1, "coordinate_units": "mm",
        "linear_deflection_mm": args.linear_deflection,
        "angular_deflection_rad": args.angular_deflection,
        "relative_deflection": False,
        "scope": "Physical sensor geometry only; no optical or FOV construction shapes",
        "sensors": {},
    }
    for name, source, count in SOURCES:
        report["sensors"][name] = convert(
            name, source, count, args.source_root.resolve(), args.output_root.resolve(),
            args.linear_deflection, args.angular_deflection)
    destination = args.output_root / "docs" / "cad_tessellation.json"
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
