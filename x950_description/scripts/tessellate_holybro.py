#!/usr/bin/env python3
"""Tessellate the official Holybro Jetson/Orin/Pixhawk6X STEP, preserving mm axes.

Requires the existing OCP Python environment. Does not download or install code.
The source provenance is recorded in holybro_jetson_baseboard.source.json.
Only closed physical solids are exported; STEP curves/sheets are inventoried.
"""
from __future__ import annotations

import json
import struct
from pathlib import Path

from OCP.Bnd import Bnd_Box
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.StlAPI import StlAPI_Writer
from OCP.TCollection import TCollection_ExtendedString
from OCP.TDataStd import TDataStd_Name
from OCP.TDF import TDF_Label, TDF_LabelSequence
from OCP.TDocStd import TDocStd_Document
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp, TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopTools import TopTools_IndexedMapOfShape
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.XCAFDoc import XCAFDoc_DocumentTool

from tessellate_sensors import clean_and_inspect_binary_stl, sha256


def remove_duplicate_facets(path):
    """Remove exactly coincident triangle records, independent of vertex order.

    The published assembly repeats some physical items in two CAD branches.
    Do not merge nearby but distinct faces or infer sameness from bounding boxes.
    """
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    seen, retained = set(), []
    for index in range(count):
        record = data[84 + 50 * index:84 + 50 * (index + 1)]
        vertices = record[12:48]
        key = b"".join(sorted((vertices[:12], vertices[12:24], vertices[24:36])))
        if key not in seen:
            seen.add(key)
            retained.append(record)
    path.write_bytes(data[:80] + struct.pack("<I", len(retained)) + b"".join(retained))
    return count - len(retained)


def bbox(shape):
    if shape.IsNull():
        return None
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    if box.IsVoid():
        return None
    values = box.Get()
    return [[round(float(x), 6) for x in values[:3]],
            [round(float(x), 6) for x in values[3:]]]


def main():
    package = Path(__file__).resolve().parents[1]
    source = package / "cad_source_meshes" / "holybro_jetson_baseboard.stp"
    output = source.with_suffix(".stl")
    reader = STEPCAFControl_Reader()
    assert reader.ReadFile(str(source)) == IFSelect_RetDone
    doc = TDocStd_Document(TCollection_ExtendedString("HolybroSource"))
    assert reader.Transfer(doc)
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    assert roots.Length() == 1
    root_shape = tool.GetShape_s(roots.Value(1))
    report = {
        "source": str(source.relative_to(package)), "source_sha256": sha256(source),
        "units": "mm", "coordinate_transform": "None; preserve native STEP assembly frame",
        "linear_deflection_mm": 0.2, "angular_deflection_rad": 0.4,
        "assembly_inventory": [], "major_component_inventory": [],
    }

    def label_name(label):
        attribute = TDataStd_Name()
        if label.FindAttribute(TDataStd_Name.GetID_s(), attribute):
            return attribute.Get().ToExtString()
        return ""

    def inventory(label, parent_location, depth):
        referred = TDF_Label()
        actual = referred if tool.GetReferredShape_s(label, referred) else label
        location = parent_location.Multiplied(tool.GetLocation_s(label))
        name = label_name(actual)
        shape = tool.GetShape_s(actual).Moved(location)
        children = TDF_LabelSequence()
        tool.GetComponents_s(actual, children)
        major = any(term in name for term in (
            "PIXHAWK6X", "V6X-", "P3668", "P3767", "GROGU_MODULE_BOARD",
            "NILONG", "SOLDERMAS", "DESIGNROOT", "095-0180", "3514_ASM",
        ))
        if depth <= 3 or major:
            row = {"name": name, "depth": depth, "children": children.Length(),
                   "bbox_mm": bbox(shape)}
            report["assembly_inventory" if depth <= 3 else "major_component_inventory"].append(row)
            print(json.dumps(row), flush=True)
        for index in range(1, children.Length() + 1):
            inventory(children.Value(index), location, depth + 1)

    inventory(roots.Value(1), TopLoc_Location(), 0)
    # IndexedMap deduplicates identical topology at identical placements, while
    # preserving repeated connectors/components at different assembly locations.
    unique_solids = TopTools_IndexedMapOfShape()
    TopExp.MapShapes_s(root_shape, TopAbs_SOLID, unique_solids)
    physical = TopoDS_Compound()
    builder = BRep_Builder()
    builder.MakeCompound(physical)
    solids = []
    placed_topologies = set()
    duplicate_topology_count = 0
    mounting_cylinders = set()
    for index in range(1, unique_solids.Extent() + 1):
        solid = unique_solids.FindKey(index)
        # The source includes the Orin assembly through two branches whose
        # placements differ only by floating-point roundoff. TopTools' exact
        # location identity misses this. Match the SAME underlying topology and
        # transform to 1e-9 mm; never deduplicate different shapes by their bounds.
        transform = solid.Location().Transformation()
        placement = tuple(round(transform.Value(r, c), 9)
                          for r in range(1, 4) for c in range(1, 5))
        placed_key = (hash(solid.Located(TopLoc_Location())), placement)
        if placed_key in placed_topologies:
            duplicate_topology_count += 1
            continue
        placed_topologies.add(placed_key)
        bounds = bbox(solid)
        dims = [bounds[1][a] - bounds[0][a] for a in range(3)]
        assert max(dims) < 200, f"Unexpected nonphysical/FOV scale: {bounds}"
        builder.Add(physical, solid)
        solids.append({"index": index, "bbox_mm": bounds, "dimensions_mm": dims})
        # Board-sized solid only: identify corner mounting bores, not connectors.
        if dims[0] > 120 and dims[1] > 75 and dims[2] < 6:
            exp = TopExp_Explorer(solid, TopAbs_FACE)
            while exp.More():
                face = TopoDS.Face_s(exp.Current())
                surface = BRepAdaptor_Surface(face, True)
                if surface.GetType() == GeomAbs_Cylinder:
                    cylinder = surface.Cylinder()
                    axis = cylinder.Axis().Direction()
                    point = cylinder.Location()
                    if abs(axis.Z()) > 0.999 and 1.0 < cylinder.Radius() < 3.0:
                        mounting_cylinders.add(tuple(round(v, 6) for v in
                            (point.X(), point.Y(), cylinder.Radius())))
                exp.Next()
    report["solids"] = solids
    report["physical_solid_count"] = len(solids)
    report["duplicate_topology_placements_removed"] = duplicate_topology_count
    report["physical_bbox_mm"] = bbox(physical)
    report["board_vertical_cylinders_xy_radius_mm"] = sorted(mounting_cylinders)
    report["non_solid_geometry"] = "Excluded STEP wires and sheets; physical closed solids retained"
    mesher = BRepMesh_IncrementalMesh(physical, 0.2, False, 0.4, True)
    assert mesher.IsDone()
    writer = StlAPI_Writer()
    writer.ASCIIMode = False
    assert writer.Write(physical, str(output))
    report["exact_duplicate_facets_removed"] = remove_duplicate_facets(output)
    report["stl"] = clean_and_inspect_binary_stl(output)
    report["output"] = str(output.relative_to(package))
    source.with_suffix(".inventory.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("physical_solid_count", "physical_bbox_mm",
          "board_vertical_cylinders_xy_radius_mm", "stl", "output")}, indent=2), flush=True)


if __name__ == "__main__":
    main()
