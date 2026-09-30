"""Add D455-derived nominal optics to D555 CAD, preserving real D555 geometry.

User confirms shared D455 optics. CAD corroborates X axes +47.5/-47.5/-11.5 mm.
D455 front-midpoint-to-depth origin (-4.65,+47.5,0) mm has an UNVERIFIED D555
axial term. No internal IMU/chip position is assumed. Live driver frames remain
separate and use device calibration. Manufacturer reference:
https://github.com/realsenseai/realsense-ros/blob/9a11121700cb4780e273e34141f6402fe184321d/realsense2_description/urdf/_d455.urdf.xacro
"""
import bpy,json,math
from pathlib import Path
from mathutils import Matrix,Vector,Euler
OUT=Path(bpy.data.filepath).resolve().parent
if OUT.name!='x950_description': OUT=OUT/'x950_description'
scene=bpy.context.scene
assert scene.name=='X950_ROS_LinkForge'
assert not any(o.name=='d555_nominal_camera_link' for o in scene.objects),'Optics already added'
housing=bpy.data.objects['d555_link']
old=bpy.data.objects.get('d555_nominal_optical_frame')
guides=list(old.children) if old else []
if old:
    old_joint=old.parent
    for g in guides:g.parent=None
    bpy.data.objects.remove(old,do_unlink=True)
    if old_joint and old_joint.linkforge_joint.is_robot_joint:bpy.data.objects.remove(old_joint,do_unlink=True)

def add(name,parent,xyz=(0,0,0),rpy=(0,0,0),datum=''):
    obj=bpy.data.objects.new(name,None);scene.collection.objects.link(obj)
    obj.empty_display_type='ARROWS';obj.empty_display_size=.018
    obj.matrix_world=parent.matrix_world@Matrix.Translation(Vector(xyz))@Euler(rpy,'XYZ').to_matrix().to_4x4()
    obj.linkforge.is_robot_link=True;obj.linkforge.link_name=name
    obj['datum']=datum;obj['calibration_status']='Nominal D455 optics in D555 CAD, not device-specific calibration'
    joint=bpy.data.objects.new(name.removesuffix('_link').removesuffix('_frame')+'_joint',None)
    scene.collection.objects.link(joint);joint.matrix_world=obj.matrix_world.copy()
    joint.empty_display_type='PLAIN_AXES';joint.empty_display_size=.012
    p=joint.linkforge_joint;p.is_robot_joint=True;p.joint_name=joint.name;p.joint_type='fixed';p.parent_link=parent;p.child_link=obj
    return obj
root=add('d555_nominal_camera_link',housing,(-.00465,.0475,0),datum='Left IR / depth origin. Lateral CAD-confirmed; -4.65mm axial offset borrowed from D455 and must be calibrated for D555.')
for sensor,xyz in [('depth',(0,0,0)),('infra1',(0,0,0)),('infra2',(0,-.095,0)),('color',(0,-.059,0))]:
    body=add('d555_nominal_'+sensor+'_frame',root,xyz,datum='D455 nominal '+sensor+' origin; lateral axis spacing corroborated by D555 CAD.')
    optical=add('d555_nominal_'+sensor+'_optical_frame',body,rpy=(-math.pi/2,0,-math.pi/2),datum='Nominal '+sensor+' optical axes: X image-right, Y image-down, Z viewing-forward.')
    for guide in guides:
        if (sensor=='depth' and 'Depth' in guide.name) or (sensor=='color' and 'RGB' in guide.name):
            guide.parent=optical;guide.matrix_parent_inverse=Matrix.Identity(4);guide.matrix_basis=Matrix.Identity(4)
            guide['apex_status']='D455-derived '+sensor+' origin; lateral CAD-confirmed, axial depth provisional.'
bpy.context.view_layer.update()
man=json.loads((OUT/'docs/geometry_manifest.json').read_text())
man['links']={o.linkforge.link_name:{'world_matrix':[list(row) for row in o.matrix_world],'datum':o.get('datum','')} for o in scene.objects if o.linkforge.is_robot_link}
man['d555_nominal_optics']={'reference':'D455 manufacturer Xacro; user states same optical module','cad_lateral_axes_mm':[47.5,-47.5,-11.5],'housing_to_depth_xyz_m':[-.00465,.0475,0],'axial_offset_status':'Provisional D455-derived estimate; D555 CAD does not establish depth zero plane.'}
(OUT/'docs/geometry_manifest.json').write_text(json.dumps(man,indent=2))
result={'links':len(man['links']),'d555_nominal_frame_count':9,'cad_confirmed_baseline_m':.095,'cad_confirmed_depth_to_rgb_m':.059,'axial_offset_provisional':True}
