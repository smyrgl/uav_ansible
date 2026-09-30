"""Reduce the avionics carrier visual inside Blender after geometric checks.

Run in the generated scene, or execute through Blender's Python console/MCP.
This does not save the .blend or export robot meshes; the calling pipeline does.
The detailed CAD source in cad_source_meshes is never changed. Only the object
avionics_carrier_visual is replaced, and only after a candidate passes checks.

Set X950_PACKAGE_ROOT to override report output. The collapse ratios are 0.08,
0.15 and 0.45, always applied to an untouched copy of the input mesh. Set
X950_OPTIMIZATION_LIBRARY_ONLY=True before exec() to load optimize_visual()
without running it, for reuse on other visual objects with explicit settings.
"""
from __future__ import annotations

from array import array
import hashlib
import json
import math
from pathlib import Path
import random
from datetime import datetime, timezone

import bpy
from mathutils.bvhtree import BVHTree


OBJECT_NAME = "avionics_carrier_visual"
MARKER = "x950_avionics_mesh_optimization_v1"
RATIOS = (0.08, 0.15, 0.45)
MAX_SAMPLES_PER_DIRECTION = 15_000
BBOX_TOLERANCE_M = 0.00025
SURFACE_TOLERANCE_M = 0.0005
TARGET_TRIANGLES = (30_000, 60_000)


def package_root():
    override = globals().get("X950_PACKAGE_ROOT")
    if override:
        return Path(override).expanduser().resolve()
    source = globals().get("__file__")
    if source and Path(source).name == "optimize_avionics.py":
        return Path(source).resolve().parents[1]
    if bpy.data.filepath:
        for parent in Path(bpy.data.filepath).resolve().parents:
            if parent.name == "x950_description":
                return parent
            candidate = parent / "x950_description"
            if (candidate / "scripts" / "optimize_avionics.py").is_file():
                return candidate
    raise RuntimeError("Cannot derive package directory; set X950_PACKAGE_ROOT")


OUT = package_root()
REPORT_PATH = OUT / "docs" / "avionics_mesh_optimization.json"


def write_report(report, path=REPORT_PATH):
    path.parent.mkdir(parents=True, exist_ok=True)
    scratch = path.with_suffix(".json.tmp")
    scratch.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    scratch.replace(path)


def fingerprint(mesh):
    """Fingerprint coordinates/topology, independent of material updates."""
    mesh.calc_loop_triangles()
    coords = array("f", [0.0]) * (3 * len(mesh.vertices))
    indices = array("i", [0]) * (3 * len(mesh.loop_triangles))
    mesh.vertices.foreach_get("co", coords)
    mesh.loop_triangles.foreach_get("vertices", indices)
    digest = hashlib.sha256()
    digest.update(coords.tobytes())
    digest.update(indices.tobytes())
    return digest.hexdigest()


def geometry(mesh, world):
    mesh.calc_loop_triangles()
    vertices = [world @ vertex.co for vertex in mesh.vertices]
    faces = [tuple(tri.vertices) for tri in mesh.loop_triangles]
    if not vertices or not faces:
        raise ValueError("Mesh contains no triangulated surface")
    lower, upper = [math.inf] * 3, [-math.inf] * 3
    for vertex in vertices:
        for axis, value in enumerate(vertex):
            if not math.isfinite(value):
                raise ValueError("Non-finite world-coordinate vertex")
            lower[axis] = min(lower[axis], value)
            upper[axis] = max(upper[axis], value)
    tree = BVHTree.FromPolygons(vertices, faces, all_triangles=True, epsilon=0.0)
    if tree is None:
        raise ValueError("Cannot build mesh surface BVH")
    return {"vertices": vertices, "faces": faces, "tree": tree,
            "bbox_m": [lower, upper], "triangle_count": len(faces)}


def surface_samples(geo, limit=MAX_SAMPLES_PER_DIRECTION):
    """Deterministic vertex coverage plus area-stratified barycentric points.

    This is sampled assurance, not a bound on every surface point. Vertices
    help retain small features; area samples cover broad faces and interiors.
    """
    vertices, faces = geo["vertices"], geo["faces"]
    vertex_count = min(len(vertices), limit // 3)
    samples = []
    if vertex_count == len(vertices):
        samples.extend((v, "vertex") for v in vertices)
    else:
        for index in range(vertex_count):
            samples.append((vertices[int((index + 0.5) * len(vertices) / vertex_count)],
                            "stratified_vertex"))
    surface_count = limit - len(samples)
    areas = []
    total = 0.0
    for a, b, c in faces:
        area = 0.5 * (vertices[b] - vertices[a]).cross(vertices[c] - vertices[a]).length
        if not math.isfinite(area):
            raise ValueError("Non-finite surface area")
        areas.append(area)
        total += area
    if total <= 0.0:
        raise ValueError("Mesh surface has zero area")
    rng = random.Random(950640)
    face_index, cumulative = 0, areas[0]
    for index in range(surface_count):
        target = (index + 0.5) * total / surface_count
        while cumulative < target and face_index < len(areas) - 1:
            face_index += 1
            cumulative += areas[face_index]
        a, b, c = (vertices[i] for i in faces[face_index])
        u = math.sqrt(rng.random())
        v = rng.random()
        point = (1.0 - u) * a + u * (1.0 - v) * b + u * v * c
        samples.append((point, "area_stratified_barycentric"))
    assert len(samples) <= limit
    return samples


def measure(samples, destination):
    distances = []
    worst = None
    for point, method in samples:
        nearest, normal, face, distance = destination["tree"].find_nearest(point)
        if nearest is None or distance is None or not math.isfinite(distance):
            raise ValueError("BVH nearest-surface query failed")
        distance = float(distance)
        distances.append(distance)
        if worst is None or distance > worst["distance_m"]:
            worst = {"distance_m": distance, "source_world_point_m": list(point),
                     "nearest_world_point_m": list(nearest),
                     "destination_triangle": int(face), "sample_method": method}
    distances.sort()
    if not distances:
        raise ValueError("No surface samples")
    return {
        "samples": len(distances), "maximum_m": distances[-1],
        "rms_m": math.sqrt(sum(d*d for d in distances) / len(distances)),
        "p95_m": distances[min(len(distances)-1, math.ceil(0.95*len(distances))-1)],
        "p99_m": distances[min(len(distances)-1, math.ceil(0.99*len(distances))-1)],
        "worst_sample": worst,
    }


def create_candidate(original_copy, world, ratio):
    """Apply a modifier via evaluated-mesh baking on an isolated object."""
    temp_mesh = original_copy.copy()
    temp = bpy.data.objects.new("__x950_avionics_decimate_candidate__", temp_mesh)
    bpy.context.scene.collection.objects.link(temp)
    temp.matrix_world = world
    temp.hide_render = True
    try:
        modifier = temp.modifiers.new("X950_Avionics_Visual_Decimate", "DECIMATE")
        modifier.decimate_type = "COLLAPSE"
        modifier.ratio = ratio
        modifier.use_collapse_triangulate = True
        modifier.use_symmetry = False
        depsgraph = bpy.context.evaluated_depsgraph_get()
        depsgraph.update()
        evaluated = temp.evaluated_get(depsgraph)
        candidate = bpy.data.meshes.new_from_object(
            evaluated, preserve_all_data_layers=True, depsgraph=depsgraph)
        if candidate is None:
            raise RuntimeError("Decimation produced no candidate mesh")
        candidate.name = "avionics_carrier_visual_optimized_mesh"
        candidate.update()
        return candidate
    finally:
        bpy.data.objects.remove(temp, do_unlink=True)
        if temp_mesh.users == 0:
            bpy.data.meshes.remove(temp_mesh)


def optimize_visual(object_name=OBJECT_NAME, ratios=RATIOS,
                    target_triangle_range=TARGET_TRIANGLES, report_path=None):
    """Optimize one visual, preserve input on failure, and return its report.

    Other objects get their own report and marker; no cumulative decimation.
    Both the bounds and sampled-surface thresholds stay fixed for every use.
    """
    report_path = (Path(report_path) if report_path is not None else
                   REPORT_PATH if object_name == OBJECT_NAME else
                   OUT / "docs" / f"{object_name}_mesh_optimization.json")
    marker = MARKER if object_name == OBJECT_NAME else "x950_visual_mesh_optimization_v1"
    if not ratios or any(not 0 < ratio < 1 for ratio in ratios):
        raise ValueError("Supply one or more collapse ratios between zero and one")
    obj = bpy.data.objects.get(object_name)
    if obj is None or obj.type != "MESH":
        raise RuntimeError(f"Required mesh object missing: {object_name}")
    if obj.mode != "OBJECT":
        raise RuntimeError("Leave mesh edit mode before optimizing the visual")
    if obj.modifiers:
        raise RuntimeError("Unexpected existing carrier modifiers; preserve and inspect them first")
    if bpy.context.scene.unit_settings.scale_length != 1.0:
        raise RuntimeError("Expected metres with scene unit scale 1.0")
    world = obj.matrix_world.copy()
    matrix = [[float(v) for v in row] for row in world]
    before_hash = fingerprint(obj.data)
    if marker in obj:
        previous = json.loads(obj[marker])
        if previous["optimized_mesh_sha256"] != before_hash:
            raise RuntimeError("Carrier mesh changed after optimization; restore detailed source before rerunning")
        old_matrix = previous["world_matrix_at_validation"]
        if any(abs(matrix[i][j] - old_matrix[i][j]) > 1e-9
               for i in range(4) for j in range(4)):
            raise RuntimeError("Carrier transform changed after validation; restore source before geometric revalidation")
        previous["last_run_was_idempotent_noop"] = True
        write_report(previous, report_path)
        return previous

    original_mesh = obj.data
    # The independent copy is the sole baseline for every retry and rollback.
    # The live object's geometry is untouched until a candidate is accepted.
    original_copy = original_mesh.copy()
    original_copy.name = "__x950_avionics_validation_original__"
    candidates = []
    accepted = None
    committed = False
    report = {
        "schema_version": 1, "object": object_name,
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "status": "evaluating", "source_mesh_sha256": before_hash,
        "world_matrix_at_validation": matrix,
        "source_preservation": "Detailed cad_source_meshes assets are untouched",
        "target_triangle_range": list(target_triangle_range),
        "bbox_component_tolerance_m": BBOX_TOLERANCE_M,
        "sampled_surface_tolerance_m": SURFACE_TOLERANCE_M,
        "maximum_samples_per_direction": MAX_SAMPLES_PER_DIRECTION,
        "validation_scope": "Actual world-vertex bounds plus bidirectional sampled nearest-surface distances; not a global Hausdorff guarantee",
        "material_preservation": "All mesh material slots must match before acceptance",
        "last_run_was_idempotent_noop": False,
        "attempts": [],
    }
    try:
        source = geometry(original_copy, world)
        report["original_triangle_count"] = source["triangle_count"]
        report["original_bbox_m"] = source["bbox_m"]
        source_samples = surface_samples(source)
        material_names = [m.name if m else None for m in original_copy.materials]
        for ratio in ratios:
            print(f"X950 {object_name}: evaluating decimation ratio {ratio}", flush=True)
            candidate = create_candidate(original_copy, world, ratio)
            candidates.append(candidate)
            reduced = geometry(candidate, world)
            deltas = [[abs(source["bbox_m"][side][axis] - reduced["bbox_m"][side][axis])
                       for axis in range(3)] for side in range(2)]
            forward = measure(source_samples, reduced)
            reverse = measure(surface_samples(reduced), source)
            same_materials = [m.name if m else None for m in candidate.materials] == material_names
            reduced_count = reduced["triangle_count"]
            passed = (max(max(row) for row in deltas) <= BBOX_TOLERANCE_M
                      and max(forward["maximum_m"], reverse["maximum_m"]) <= SURFACE_TOLERANCE_M
                      and same_materials and 0 < reduced_count < source["triangle_count"])
            attempt = {
                "collapse_ratio": ratio, "triangle_count": reduced_count,
                "vertex_count": len(reduced["vertices"]),
                "within_triangle_target": target_triangle_range[0] <= reduced_count <= target_triangle_range[1],
                "bbox_m": reduced["bbox_m"], "bbox_absolute_delta_m": deltas,
                "source_to_reduced": forward, "reduced_to_source": reverse,
                "mesh_material_slots_preserved": same_materials, "accepted": passed,
            }
            report["attempts"].append(attempt)
            print(f"X950 {object_name}: {reduced_count} triangles; "
                  f"bounds error {max(max(row) for row in deltas)*1000:.4f} mm; "
                  f"sampled errors {forward['maximum_m']*1000:.4f}/"
                  f"{reverse['maximum_m']*1000:.4f} mm; accepted={passed}", flush=True)
            if passed:
                accepted = candidate
                report["selected_attempt"] = len(report["attempts"]) - 1
                break
        if accepted is None:
            raise RuntimeError(f"All {object_name} decimation candidates exceeded geometric limits; original retained")
        report["status"] = "passed"
        report["optimized_triangle_count"] = report["attempts"][-1]["triangle_count"]
        report["optimized_mesh_sha256"] = fingerprint(accepted)
        report["warning"] = (None if report["attempts"][-1]["within_triangle_target"]
                             else f"Accepted result falls outside the {target_triangle_range[0]}-{target_triangle_range[1]} triangle target; geometric accuracy checks passed")
        accepted.name = f"{object_name}_optimized_mesh"
        obj.data = accepted
        obj[marker] = json.dumps(report, allow_nan=False)
        write_report(report, report_path)
        committed = True
        return report
    except Exception as exc:
        obj.data = original_mesh
        if marker in obj:
            del obj[marker]
        report["status"] = "failed_original_restored"
        report["error"] = str(exc)
        try:
            write_report(report, report_path)
        except Exception as report_error:
            print(f"Could not write failure report: {report_error}", flush=True)
        raise
    finally:
        for mesh in candidates:
            if (mesh != accepted or not committed) and mesh.users == 0:
                bpy.data.meshes.remove(mesh)
        if original_copy.users == 0:
            bpy.data.meshes.remove(original_copy)


if not globals().get("X950_OPTIMIZATION_LIBRARY_ONLY", False):
    result = optimize_visual()
    print(json.dumps({"status": result["status"],
                      "original_triangles": result["original_triangle_count"],
                      "optimized_triangles": result["optimized_triangle_count"],
                      "report": str(REPORT_PATH),
                      "warning": result.get("warning"),
                      "idempotent_noop": result["last_run_was_idempotent_noop"]}, indent=2))
