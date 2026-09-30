#!/usr/bin/env python3
"""Create a provisional Hadron 640R+ visual envelope, NOT manufacturer CAD.

The official STEP is supplied on request. This simple mesh uses the public
Hadron 640 series R170 drawing dimensions; lens details are illustrative.
Coordinates are millimetres in an upright local FLU frame, with origin at
the rear mounting-plane centre. Mount with a 180-degree roll for thermal-down.
Uses only Python's standard library and never edits the live Blender scene.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import struct

PACKAGE = Path(__file__).resolve().parents[1]
OUT = PACKAGE / "cad_source_meshes"


def rounded_profile(x, width, height, radius, segments=8):
    """Counterclockwise as viewed from +X, with coordinates (X, Y, Z)."""
    points = []
    corners = ((width/2-radius, height/2-radius, 0),
               (-width/2+radius, height/2-radius, 90),
               (-width/2+radius, -height/2+radius, 180),
               (width/2-radius, -height/2+radius, 270))
    for y, z, angle in corners:
        for i in range(segments+1):
            a = math.radians(angle+90*i/segments)
            points.append((x, y+radius*math.cos(a), z+radius*math.sin(a)))
    return points


def loft(profiles):
    triangles = []
    n = len(profiles[0])
    for a, b in zip(profiles, profiles[1:]):
        for i in range(n):
            j = (i+1) % n
            triangles.extend(((a[i], a[j], b[j]), (a[i], b[j], b[i])))
    for profile, positive in ((profiles[0], False), (profiles[-1], True)):
        centre = tuple(sum(p[i] for p in profile)/n for i in range(3))
        for i in range(n):
            a, b = profile[i], profile[(i+1) % n]
            triangles.append((centre, a, b) if positive else (centre, b, a))
    return triangles


def cylinder(x0, x1, radius, z, n=64):
    rings = [[(x, radius*math.cos(2*math.pi*i/n),
               z+radius*math.sin(2*math.pi*i/n)) for i in range(n)]
             for x in (x0, x1)]
    return loft(rings)


def write_stl(name, triangles):
    data = bytearray(b"Hadron 640R+ DRAWING PROXY; millimetres; upright FLU".ljust(80,b" "))
    data.extend(struct.pack("<I",len(triangles)))
    lower, upper = [math.inf]*3, [-math.inf]*3
    for a,b,c in triangles:
        ab = [b[i]-a[i] for i in range(3)]
        ac = [c[i]-a[i] for i in range(3)]
        normal = [ab[1]*ac[2]-ab[2]*ac[1], ab[2]*ac[0]-ab[0]*ac[2], ab[0]*ac[1]-ab[1]*ac[0]]
        length = math.sqrt(sum(v*v for v in normal))
        if length < 1e-12:
            raise ValueError("Degenerate triangle")
        normal = [v/length for v in normal]
        data.extend(struct.pack("<12fH",*normal,*a,*b,*c,0))
        for vertex in (a,b,c):
            for i,v in enumerate(vertex):
                lower[i] = min(lower[i],v)
                upper[i] = max(upper[i],v)
    path = OUT/name
    path.write_bytes(data)
    return {"path":str(path.relative_to(PACKAGE)),"triangles":len(triangles),
            "bbox_mm":[lower,upper],"sha256":hashlib.sha256(data).hexdigest()}


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    body = loft([rounded_profile(0,36,50,2),
                 rounded_profile(30,35,49,2),
                 rounded_profile(40.2,30,43,2)])
    gasket = loft([rounded_profile(-0.5,35,49,2),rounded_profile(0,35,49,2)])
    # Lens centre spacing and barrel radii are visual estimates from Fig. 4.
    # Front lens surfaces are NOT measured entrance pupils or sensor planes.
    thermal = cylinder(38.5,42.65,13.25,7.8)
    visible = cylinder(38.5,41.65,4.8,-12.2)
    files = {}
    for name,triangles in (("body",body+gasket),("thermal_lens",thermal),
                           ("visible_lens",visible),
                           ("complete",body+gasket+thermal+visible)):
        files[name] = write_stl(f"hadron_640r_plus_proxy_{name}.stl",triangles)
    sources = {
        "model":"FLIR Hadron 640R+", "part_number":"70640AS32-6PARXP",
        "geometry_status":"Drawing-derived visual proxy, not official STEP/CAD",
        "source_date":"2026-09-24",
        "product_url":"https://oem.flir.com/products/hadron-640/?model=70640AS32-6PARXP",
        "engineering_drawing_url":"https://flir.netx.net/file/asset/52567/original/attachment/",
        "engineering_drawing":"102-2053-40 R170; Fig. 3 and Fig. 4, PDF pages 9-10",
        "cad_availability_url":"https://flir.custhelp.com/app/answers/detail/a_id/6003/~/flir-oem---cores-support",
        "cad_request_url":"https://flir.custhelp.com/app/ask",
        "cad_request_status":"Not submitted; official support table says Available upon Request",
        "units":"mm",
        "frame":{"origin":"Centre of rear housing mounting plane, excluding 0.5 mm gasket",
                 "x":"Forward along viewing direction", "y":"Left", "z":"Up in factory upright orientation",
                 "aircraft_mount":"Apply roll pi around local X for thermal below visible; do not flip twice"},
        "dimensioned_body_extents_xyz_mm":[42.65,36,50],
        "gasket_extra_rear_depth_mm":0.5,
        "mass_kg":0.056,
        "mass_status":"Manufacturer nominal; no mounting bracket, cables or hardware",
        "cog_xyz_mm":[19.125,0,0.98],
        "cog_status":"Approximate manufacturer COG: 19.125 mm from rear datum and 6.82 mm below thermal axis; lateral 0.019 mm drawing offset ignored",
        "thermal_axis_height_mm":7.8,
        "thermal_axis_height_status":"Derived: 50/2 - [(50-41.6)/2 + 13]",
        "thermal_visual_surface_xyz_mm":[42.65,0,7.8],
        "visible_visual_surface_xyz_mm":[41.65,0,-12.2],
        "optical_origin_status":"Visual lens-face references only, not calibrated optical centres/entrance pupils; visible 20 mm spacing estimated from drawing",
        "thermal_resolution":[640,512], "thermal_hfov_deg":32,
        "visible_resolution":[9248,6944], "visible_hfov_deg":67,
        "thermal_vfov_estimated_deg":math.degrees(2*math.atan(math.tan(math.radians(16))*512/640)),
        "visible_vfov_estimated_deg":math.degrees(2*math.atan(math.tan(math.radians(33.5))*6944/9248)),
        "vfov_status":"Derived pinhole full-sensor estimate; not a measured/manufacturer vertical FOV",
        "omissions":["Mounting holes and rear connectors", "Internal solids", "Exact lens profile and optical centre depths", "Calibrated camera alignment", "Actual inertia tensor"],
        "files":files,
    }
    (OUT/"hadron_640r_plus_proxy.source.json").write_text(json.dumps(sources,indent=2)+"\n")
    print(json.dumps({"geometry_status":sources["geometry_status"],"files":files},indent=2))


if __name__ == "__main__":
    main()
