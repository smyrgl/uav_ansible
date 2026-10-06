"""Apply the A-S+ nose bracket set (Pitch_Study integration, 2026-10-05) to the generated scene.

Avia cage v2 (45 deg nose-down), D555 saddle v0.4 (40 deg nose-down, inverted) and
Hadron base v2 + carrier v1 (15 deg): joint origins from
Printables/Pitch_Study/build/urdf_poses.json, bracket visuals from the builders'
aircraft-frame STLs. Run through blender_rpc.py with the generated scene active.
The sensor links, their optical children and the Hadron plate v2 ride on the
joints unchanged; only the mount brackets and the three joint origins change.
"""
import bpy, json, math
from pathlib import Path
from mathutils import Matrix, Vector, Euler

p = Path(bpy.data.filepath).resolve().parent
if p.name != 'x950_description':
    p = p / 'x950_description'
exec(compile((p / 'scripts/scene_helpers.py').read_text(), str(p / 'scripts/scene_helpers.py'), 'exec'))

ACF = Matrix(((0, -.001, 0, 0), (-.001, 0, 0, 0), (0, 0, -.001, 0), (0, 0, 0, 1)))   # aircraft mm -> ROS m
PRINTABLES = Path('/Users/john/Code/Printables')
FLIR = Path('/Users/john/Code/KiCAD/FLIR/mount')
POSES = json.loads((PRINTABLES / 'Pitch_Study/build/urdf_poses.json').read_text())
HADRON_TILT = globals().get('HADRON_TILT', '15deg')
paht = bpy.data.materials['X950_finish_paht_cf_black']
SOURCE_TAG = 'A-S+ nose integration (Printables/Pitch_Study/build, 2026-10-05); nominal CAD pose of an unbuilt bracket: calibrate after the build'


def set_joint(joint, parent, xyz, rpy):
    j = scene.objects[joint]
    pl = scene.objects[parent]
    assert j.linkforge_joint.parent_link == pl, (joint, parent)
    j.matrix_world = pl.matrix_world @ Matrix.Translation(Vector(xyz)) @ Euler(rpy, 'XYZ').to_matrix().to_4x4()
    bpy.context.view_layer.update()


def replace_visuals(anchor_name, remove, parts):
    anchor = scene.objects[anchor_name]
    for n in remove:
        o = scene.objects.get(n)
        if o is None:
            continue
        me = o.data
        bpy.data.objects.remove(o, do_unlink=True)
        if me is not None and me.users == 0:
            bpy.data.meshes.remove(me)
    T = anchor.matrix_world.inverted() @ ACF
    made = []
    for path, label in parts:
        path = Path(path)
        assert path.exists(), path
        o = import_stl(path, anchor, T, paht, label + '_visual')
        o['geometry_status'] = 'Printed bracket, builder STL %s; %s' % (path.name, SOURCE_TAG)
        made.append(o.name)
    return made


# --- joints -------------------------------------------------------------------
a = POSES['avia']['urdf_avia_joint']
set_joint('avia_joint', 'avia_mount_link', a['xyz_m'], a['rpy'])
d = POSES['d555']['urdf_d555_joint']
set_joint('d555_joint', 'd555_mount_link', d['xyz_m'], d['rpy'])
h = POSES['hadron'][HADRON_TILT]['urdf_hadron_mount_joint']
set_joint('hadron_mount_joint', 'base_link', h['xyz_m'], h['rpy'])

# --- bracket visuals ------------------------------------------------------------
made = {}
made['avia'] = replace_visuals('avia_mount_link', ['avia_bracket_visual', 'avia_backing_visual'], [
    (PRINTABLES / 'Avia/avia_cage_v2_main_aircraft.stl', 'avia_cage_v2_main'),
    (PRINTABLES / 'Avia/avia_cage_v2_pad_strut_aircraft.stl', 'avia_cage_v2_pad_strut'),
    (PRINTABLES / 'Avia/avia_cage_v2_top_shim_t1.6_aircraft.stl', 'avia_cage_v2_top_shim'),
    (PRINTABLES / 'Avia/avia_jack_foot_v2_aircraft.stl', 'avia_jack_foot_v2'),
    (PRINTABLES / 'Avia/avia_backing_plate_v2_aircraft.stl', 'avia_backing_plate_v2'),
])
made['d555'] = replace_visuals('d555_mount_link', ['d555_bracket_v03_visual'], [
    (PRINTABLES / 'D555/D555_saddle_v04_aircraft.stl', 'd555_saddle_v04'),
])
made['hadron'] = replace_visuals('hadron_mount_link',
    ['hadron_wedge_v1_visual', 'hadron_wedge_backing_lower_visual', 'hadron_wedge_backing_upper_visual'], [
    (FLIR / 'base_v2/base_v2_air.stl', 'hadron_base_v2'),
    (FLIR / 'base_v2/backing_block_v2_lower_air.stl', 'hadron_backing_block_v2_lower'),
    (FLIR / 'base_v2/backing_block_v2_upper_air.stl', 'hadron_backing_block_v2_upper'),
    (FLIR / ('carrier_v1/carrier_v1_%s_air.stl' % HADRON_TILT), 'hadron_carrier_v1_' + HADRON_TILT),
])

# --- datums -----------------------------------------------------------------------
scene.objects['avia_link']['datum'] = ('Avia bottom mounting-pattern centroid in the A-S+ cage v2, 45 deg nose-down; '
    '+X forward, +Y left, +Z up. Not the Livox ranging origin (runtime avia_lidar_* puts the bezel plane at +0.0525 X, +0.0324 Z). ' + SOURCE_TAG)
scene.objects['d555_link']['datum'] = ('D555 front housing-plane centre in saddle v0.4, inverted and 40 deg nose-down; '
    '+X forward, +Y right, +Z down. Not a calibrated imager origin. ' + SOURCE_TAG)
scene.objects['hadron_mount_link']['datum'] = ('Rear housing face centre (lip-top plane) on base v2 + carrier v1 (%s): xyz %s m, rpy %s; '
    '+X forward, +Y left, +Z up. ' % (HADRON_TILT, h['xyz_m'], h['rpy']) + SOURCE_TAG)
scene.objects['hadron_mount_link']['placement_status'] = 'Nominal CAD pose of the A-S+ bracket set (2026-10-05); calibrate after the build'

man = refresh_manifest()
bpy.ops.wm.save_mainfile()

def w(name):
    return [round(v, 6) for v in scene.objects[name].matrix_world.translation]
def rpy(name):
    return [round(math.degrees(v), 3) for v in scene.objects[name].matrix_world.to_euler('XYZ')]
check = {
    'avia_link': w('avia_link'), 'avia_link_rpy_deg': rpy('avia_link'),
    'd555_link': w('d555_link'), 'd555_link_rpy_deg': rpy('d555_link'),
    'hadron_link': w('hadron_link'), 'hadron_thermal_optical': w('hadron_nominal_thermal_optical_frame'),
    'expected': {'avia_link': POSES['avia']['avia_link_in_base_link']['xyz_m'],
                 'd555_link': POSES['d555']['d555_link_in_base_link']['xyz_m'],
                 'hadron_thermal_optical': POSES['hadron'][HADRON_TILT]['thermal_optical_in_base_link']['xyz_m']},
    'new_visuals': made,
    'manifest': {'links': len(man['links']), 'visual_meshes': len(man['meshes'])},
    'bounds_m': {m['name']: m['bounds_m'] for m in man['meshes'] if any(k in m['name'] for k in ('cage', 'saddle', 'carrier', 'base_v2', 'backing_block', 'jack', 'top_shim', 'backing_plate'))},
    'saved': bpy.data.filepath,
}
print(json.dumps(check, indent=1))
