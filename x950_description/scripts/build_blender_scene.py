"""Build a separate ROS-FLU scene from the open x950.blend and prepared sensor STLs.

Run through blender_rpc.py. Source scene and source .blend are not overwritten.
All coordinates in the generated scene are metres; all mesh scales are baked.
"""
import bpy
import json
import math
from pathlib import Path
from mathutils import Matrix, Vector

ROOT = Path(bpy.data.filepath).resolve().parent
OUT = ROOT / 'x950_description'
SCENE_NAME = 'X950_ROS_LinkForge'
assert SCENE_NAME not in bpy.data.scenes, 'Generated scene already exists; inspect before rebuilding.'
source = bpy.context.scene
assert len([o for o in source.objects if o.type == 'MESH']) == 26, 'Unexpected source scene'
for sensor in ('avia','d555','e1r'):
    assert (OUT / 'cad_source_meshes' / ('source_' + sensor + '.stl')).is_file()

scene = bpy.data.scenes.new(SCENE_NAME)
bpy.context.window.scene = scene
scene.unit_settings.system = 'METRIC'
scene.unit_settings.scale_length = 1.0
scene['description_scope'] = 'Mechanical geometry and fixed transforms only; no calibrated driver extrinsics or dynamics.'
scene['source_scene'] = source.name
scene['frame_convention'] = 'ROS FLU: +X forward, +Y left, +Z up; origin is original aircraft CAD datum.'
scene['export_instruction'] = 'Run scripts/export_linkforge.py; stock export invents default mass/inertia.'

ACF = Matrix(((0,-.001,0,0),(-.001,0,0,0),(0,0,-.001,0),(0,0,0,1)))
BLEND_TO_ROS = Matrix(((0,0,-1,0),(-1,0,0,0),(0,1,0,0),(0,0,0,1)))

materials={}
def material(name,rgba,metallic=0.0):
    m=bpy.data.materials.new(name)
    m.diffuse_color=rgba
    m.use_nodes=True
    bs=m.node_tree.nodes.get('Principled BSDF')
    bs.inputs['Base Color'].default_value=rgba
    bs.inputs['Metallic'].default_value=metallic
    bs.inputs['Roughness'].default_value=.42
    materials[name]=m
    return m

body_mat=material('x950_graphite',(.18,.22,.28,1),.2)
arm_mat=material('x950_carbon',(.075,.09,.12,1),.15)
cover_mat=material('x950_shell',(.46,.51,.57,1),.1)
mount_mat=material('sensor_mount_orange',(.94,.30,.055,1))
avia_mat=material('avia_teal',(.04,.58,.64,1),.2)
d555_mat=material('d555_blue',(.10,.31,.72,1),.2)
e1r_mat=material('e1r_green',(.18,.57,.34,1),.2)

links={}
def link(name,xyz=(0,0,0),rpy=(0,0,0),datum=''):
    from mathutils import Euler
    o=bpy.data.objects.new(name,None)
    scene.collection.objects.link(o)
    o.empty_display_type='ARROWS'
    o.empty_display_size=.04
    o.matrix_world=Matrix.Translation(Vector(xyz)) @ Euler(rpy,'XYZ').to_matrix().to_4x4()
    o.linkforge.is_robot_link=True
    o.linkforge.link_name=name
    o['datum']=datum
    o['mass_status']='Not measured; excluded by export_linkforge.py'
    links[name]=o
    return o

def fixed(parent,child):
    o=bpy.data.objects.new(child.name.removesuffix('_link')+'_joint',None)
    scene.collection.objects.link(o)
    o.matrix_world=child.matrix_world.copy()
    o.empty_display_type='PLAIN_AXES'
    o.empty_display_size=.025
    p=o.linkforge_joint
    p.is_robot_joint=True
    p.joint_name=o.name
    p.joint_type='fixed'
    p.parent_link=parent
    p.child_link=child
    return o

def prepare_mesh(o,anchor,T,mat,label):
    o.parent=None
    o.data.transform(anchor.matrix_world.inverted() @ T @ o.matrix_world)
    o.matrix_world=Matrix.Identity(4)
    o.parent=anchor
    o.matrix_parent_inverse=Matrix.Identity(4)
    o.matrix_basis=Matrix.Identity(4)
    o.name=label+'_visual'
    o.linkforge_geom.geometry_type='mesh'
    o.data.materials.clear()
    o.data.materials.append(mat)
    return o

def import_stl(path,anchor,T,mat,label):
    before=set(scene.objects)
    bpy.ops.wm.stl_import(filepath=str(path),global_scale=1.0,use_scene_unit=False)
    items=[o for o in scene.objects if o not in before and o.type=='MESH']
    assert len(items)==1,(path,len(items))
    return prepare_mesh(items[0],anchor,T,mat,label)

base=link('base_link',datum='Aircraft CAD origin, preserved from source; not calibrated to autopilot IMU or CoM.')
for o in list(source.objects):
    if o.type!='MESH': continue
    dup=o.copy();dup.data=o.data.copy()
    dup.parent=None;dup.matrix_world=o.matrix_world.copy()
    scene.collection.objects.link(dup)
    mat=arm_mat if ('motor' in o.name or 'arm' in o.name) else cover_mat if ('cover' in o.name or 'door' in o.name) else body_mat
    prepare_mesh(dup,base,BLEND_TO_ROS,mat,'x950_'+o.name)

avia_mount=link('avia_mount_link',(.14766,0,.09325),datum='Centre of Avia sandwich bolt field: ACF (0,-147.66,-93.25) mm; nominal bracket datum.')
avia=link('avia_link',(.20966,-.002525,.078),datum='Avia bottom mounting-pattern centroid; +X forward,+Y left,+Z up. Not Livox ranging origin.')
d555_mount=link('d555_mount_link',(.104904,0,-.0265),datum='Front belly plate mounting rectangle centre, ACF (0,-104.904,26.5) mm.')
d555=link('d555_link',(.150,0,-.06935),(math.pi,0,0),datum='D555 CAD front housing-plane centre; mounted inverted (+X forward,+Y right,+Z down). Not calibrated imager origin.')
e1r_mount=link('e1r_mount_link',(-.084,0,-.0245),datum='Rear belly plate bolt-pattern centre, ACF (0,84,24.5) mm.')
e1r=link('e1r_link',(-.084,0,-.0849),datum='E1R supplied CAD datum: X wide axis forward, Y left, Z rear of sensor/up; viewing direction -Z.')
for p,c in ((base,avia_mount),(avia_mount,avia),(base,d555_mount),(d555_mount,d555),(base,e1r_mount),(e1r_mount,e1r)):
    fixed(p,c)

# CAD/source → aircraft transforms in millimetres.
Avia = Matrix(((0,0,-1,-35.975),(1,0,0,-281.960),(0,-1,0,-62.100),(0,0,0,1)))
D555 = Matrix(((1,0,0,0),(0,0,-1,-150),(0,1,0,69.350),(0,0,0,1)))
E1R = Matrix(((0,-1,0,0),(-1,0,0,84),(0,0,-1,84.9),(0,0,0,1)))
D555br=Matrix(((-1,0,0,0),(0,-1,0,-92),(0,0,1,26.5),(0,0,0,1)))
E1Rbr=Matrix.Translation((0,84,24.5))
for rel,a,T,m,label in (
 ('Avia/avia_mount_bracket.stl',avia_mount,ACF,mount_mat,'avia_bracket'),
 ('Avia/avia_backing_plate.stl',avia_mount,ACF,mount_mat,'avia_backing'),
 ('D555/D555_belly_bracket_v03.stl',d555_mount,ACF@D555br,mount_mat,'d555_bracket_v03'),
 ('E1R/E1R_belly_bracket_v01.stl',e1r_mount,ACF@E1Rbr,mount_mat,'e1r_bracket_v01'),
 ('x950_description/cad_source_meshes/source_avia.stl',avia,ACF@Avia,avia_mat,'avia_sensor'),
 ('x950_description/cad_source_meshes/source_d555.stl',d555,ACF@D555,d555_mat,'d555_sensor'),
 ('x950_description/cad_source_meshes/source_e1r.stl',e1r,ACF@E1R,e1r_mat,'e1r_sensor')):
    import_stl(ROOT/rel,a,T,m,label)

# Explicitly nominal frames avoid masquerading as driver calibration.
optical=link('d555_nominal_optical_frame',datum='Nominal optical axes at front housing centre, NOT an actual depth/color optical centre.')
from mathutils import Euler
optical.matrix_world=d555.matrix_world @ Euler((-math.pi/2,0,-math.pi/2),'XYZ').to_matrix().to_4x4()
fixed(d555,optical)
e1r_ranging=link('e1r_nominal_lidar_frame',(-.084,0,-.079312),(0,math.pi/2,0),datum='CAD modeled optic centre, X down/beam, Y left, Z forward; verify driver convention/calibration before data fusion.')
fixed(e1r,e1r_ranging)

for sensor,kg in {'avia':.498,'d555':.337,'e1r':.330}.items():
    o=links[sensor+'_link']
    o.linkforge.mass=kg
    o['mass_kg']=kg
    o['mass_status']='User supplied 2026-09-24; inertia not established; stored as metadata.'

links['d555_link']['depth_fov_degrees']=[87.,58.]
links['d555_link']['rgb_fov_degrees']=[90.,65.]
links['d555_link']['fov_tolerance_degrees']=3.
links['d555_link']['depth_fov_mode']='HD 16:9'

scene.linkforge_robot.robot_name='x950'
scene.linkforge_robot.mesh_format='STL'
scene.linkforge_robot.use_ros2_control=False
bpy.context.view_layer.update()
for o in scene.objects:
    o.select_set(False)
base.select_set(True)
bpy.context.view_layer.objects.active=base

# A readable review view; rendering is performed by a separate script.
from mathutils import Quaternion
for area in bpy.context.screen.areas:
    if area.type=='VIEW_3D':
        space=area.spaces.active
        space.clip_end=100
        space.shading.type='SOLID'
        space.shading.color_type='MATERIAL'
        space.region_3d.view_distance=1.8
        space.region_3d.view_location=(0,0,-.02)
        space.region_3d.view_rotation=Vector((1.4,-1.8,1.1)).to_track_quat('Z','Y')

manifest={
 'source_blend':str(ROOT/'x950.blend'), 'scene':scene.name,
 'units':'metres','base_origin':'original aircraft CAD datum; not CoM/IMU',
 'source_aircraft_mm_to_ros_m':[list(row) for row in ACF],
 'source_blender_m_to_ros_m':[list(row) for row in BLEND_TO_ROS],
 'sensor_source_to_aircraft_mm':{'avia':[list(row) for row in Avia],'d555':[list(row) for row in D555],'e1r':[list(row) for row in E1R]},
 'links':{n:{'world_matrix':[list(row) for row in o.matrix_world],'datum':o['datum']} for n,o in links.items()},
 'meshes':[]}
for o in scene.objects:
    if o.type!='MESH':continue
    ps=[o.matrix_world@Vector(p) for p in o.bound_box]
    manifest['meshes'].append({'name':o.name,'parent':o.parent.name,'faces':len(o.data.polygons),'bounds_m':[[min(v[k] for v in ps) for k in range(3)],[max(v[k] for v in ps) for k in range(3)]]})
(OUT/'docs/geometry_manifest.json').write_text(json.dumps(manifest,indent=2))
for step in ('apply_d555_nominal_optics.py','add_coverage_guides.py','add_battery.py','add_support_hardware.py','add_hadron.py','apply_materials.py','optimize_avionics.py','optimize_display.py'):
    path=OUT/'scripts'/step
    exec(compile(path.read_text(),str(path),'exec'),{'__name__':'__main__'})
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'x950_linkforge.blend'))
result={'scene':scene.name,'links':sum(o.linkforge.is_robot_link for o in scene.objects),'meshes':sum(o.type=='MESH' and '_visual' in o.name for o in scene.objects),'saved':bpy.data.filepath}
