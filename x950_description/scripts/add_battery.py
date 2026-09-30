"""Add the measured battery envelope and printed TPU support envelopes.

Source: Battery/cradle.py, end_block.py, bay-geometry-verified.md and battery
handoff. Battery source frame is millimetres X starboard, Y up, Z aft (NOT the
Avia/D555/E1R aircraft frame). The STL represents external envelopes, not slicer
infill. Pack CoM/inertia are unknown; geometric centre/uniform-box estimates are
recorded as estimates in config/mass_properties.json, not exported as physics.
"""
import bpy,json
from pathlib import Path
from mathutils import Matrix,Vector
OUT=Path(bpy.data.filepath).resolve().parent
if OUT.name!='x950_description':OUT=OUT/'x950_description'
ROOT=OUT.parent
scene=bpy.context.scene
assert scene.name=='X950_ROS_LinkForge'
assert 'battery_link' not in scene.objects,'Battery already exists'
A=Matrix(((0,0,-.001,0),(-.001,0,0,0),(0,.001,0,0),(0,0,0,1)))
base=bpy.data.objects['base_link']

def anchor(name,xyz,parent,datum):
    o=bpy.data.objects.new(name,None);scene.collection.objects.link(o)
    o.matrix_world=Matrix.Translation(Vector(xyz));o.empty_display_type='ARROWS';o.empty_display_size=.025
    o.linkforge.is_robot_link=True;o.linkforge.link_name=name;o['datum']=datum
    j=bpy.data.objects.new(name.removesuffix('_link')+'_joint',None);scene.collection.objects.link(j)
    j.matrix_world=o.matrix_world.copy();j.empty_display_type='PLAIN_AXES';j.empty_display_size=.018
    p=j.linkforge_joint;p.is_robot_joint=True;p.joint_name=j.name;p.joint_type='fixed';p.parent_link=parent;p.child_link=o
    return o

def material(name,color):
    m=bpy.data.materials.new(name);m.diffuse_color=color;return m

support=anchor('battery_support_link',(-.02623,0,-.010),base,'Floor datum beneath pack geometric centre; TPU parts shown in their uncompressed source envelopes.')
support['mass_status']='Printed mass unknown; source infill dependent estimate not counted.'
pack=anchor('battery_link',(-.02623,0,.03410),support,'Measured pack envelope centre from final cradle design. Geometric centre, not measured centre of mass.')
pack.linkforge.mass=3.06;pack['mass_kg']=3.06;pack['mass_status']='3060 g from battery handoff; CoM and inertia not measured.'
pack['model']='MAD Components semi-solid 12S 20Ah';pack['dimensions_source_mm']=[103.6,72.6,191.5]
pack['pack_pose_status']='Nominal uncompressed cradle CAD; 0.4 mm intended vertical preload.'
packmat=material('battery_pack_burgundy',(.46,.045,.10,1));tpumat=material('battery_TPU_sand',(.66,.49,.25,1))
for name in ('cradle','end_block'):
    before=set(scene.objects)
    bpy.ops.wm.stl_import(filepath=str(ROOT/'Battery/stl'/(name+'.stl')),global_scale=1,use_scene_unit=False)
    o=next(o for o in scene.objects if o not in before and o.type=='MESH')
    o.data.transform(support.matrix_world.inverted()@A@o.matrix_world)
    o.matrix_world=Matrix.Identity(4);o.parent=support;o.matrix_parent_inverse=Matrix.Identity(4);o.matrix_basis=Matrix.Identity(4)
    o.name='battery_'+name+'_visual';o.linkforge_geom.geometry_type='mesh';o.data.materials.append(tpumat)
    o['geometry_status']='Printed-part envelope only; infill lattice absent from source STL.'
bpy.ops.mesh.primitive_cube_add(size=1)
o=bpy.context.object;o.name='battery_pack_visual'
o.data.transform(Matrix.Diagonal((.1915,.1036,.0726,1)))
o.parent=pack;o.matrix_parent_inverse=Matrix.Identity(4);o.matrix_basis=Matrix.Identity(4)
o.linkforge_geom.geometry_type='mesh';o.data.materials.append(packmat)
o['geometry_status']='Box envelope from measured pack dimensions; leads and recessed rear end omitted.'
bpy.context.view_layer.update()
man=json.loads((OUT/'docs/geometry_manifest.json').read_text())
man['links']={o.linkforge.link_name:{'world_matrix':[list(row) for row in o.matrix_world],'datum':o.get('datum','')} for o in scene.objects if o.linkforge.is_robot_link}
for o in scene.objects:
    if o.type!='MESH' or not o.name.startswith('battery_'):continue
    ps=[o.matrix_world@Vector(p) for p in o.bound_box]
    man['meshes'].append({'name':o.name,'parent':o.parent.name,'faces':len(o.data.polygons),'bounds_m':[[min(v[k] for v in ps) for k in range(3)],[max(v[k] for v in ps) for k in range(3)]]})
man['battery']={'mass_kg':3.06,'source_dimensions_mm':[103.6,72.6,191.5],'ros_dimensions_m':[.1915,.1036,.0726],'geometric_center_ros_m':[-.02623,0,.0341],'center_of_mass_status':'Not measured; geometric centre only','source_to_ros_matrix':[list(row) for row in A]}
(OUT/'docs/geometry_manifest.json').write_text(json.dumps(man,indent=2))
result={'battery_mass_kg':3.06,'links':len(man['links']),'visual_meshes':len(man['meshes']),'printed_parts':['cradle','end_block'],'excluded':['coupon_floor','coupon_side'],'internal_infill_not_available':True}
