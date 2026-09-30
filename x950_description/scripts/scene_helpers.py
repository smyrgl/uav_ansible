"""Utilities for standalone Blender authoring scripts; no scene mutations on load."""
import bpy,json,math
from pathlib import Path
from mathutils import Matrix,Vector,Euler
OUT=Path(bpy.data.filepath).resolve().parent
if OUT.name!='x950_description':OUT=OUT/'x950_description'
ROOT=OUT.parent
scene=bpy.context.scene
assert scene.name=='X950_ROS_LinkForge'

def material(name,rgba):
    m=bpy.data.materials.get(name) or bpy.data.materials.new(name)
    m.diffuse_color=rgba
    return m

def link(name,parent,relative_xyz=(0,0,0),relative_rpy=(0,0,0),datum=''):
    assert name not in scene.objects,name
    o=bpy.data.objects.new(name,None);scene.collection.objects.link(o)
    o.matrix_world=parent.matrix_world@Matrix.Translation(Vector(relative_xyz))@Euler(relative_rpy,'XYZ').to_matrix().to_4x4()
    o.empty_display_type='ARROWS';o.empty_display_size=.018
    o.linkforge.is_robot_link=True;o.linkforge.link_name=name;o['datum']=datum
    j=bpy.data.objects.new(name.removesuffix('_link')+'_joint',None);scene.collection.objects.link(j)
    j.matrix_world=o.matrix_world.copy();j.empty_display_type='PLAIN_AXES';j.empty_display_size=.012
    p=j.linkforge_joint;p.is_robot_joint=True;p.joint_name=j.name;p.joint_type='fixed';p.parent_link=parent;p.child_link=o
    bpy.context.view_layer.update()
    return o

def adopt_mesh(o,anchor,local_transform,mat,name):
    o.data.transform(local_transform@o.matrix_world)
    o.matrix_world=Matrix.Identity(4);o.parent=anchor
    o.matrix_parent_inverse=Matrix.Identity(4);o.matrix_basis=Matrix.Identity(4)
    o.name=name;o.linkforge_geom.geometry_type='mesh';o.data.materials.clear();o.data.materials.append(mat)
    return o

def import_stl(path,anchor,local_transform,mat,name):
    before=set(scene.objects)
    bpy.ops.wm.stl_import(filepath=str(path),global_scale=1,use_scene_unit=False)
    objects=[o for o in scene.objects if o not in before and o.type=='MESH']
    assert len(objects)==1
    return adopt_mesh(objects[0],anchor,local_transform,mat,name)

def refresh_manifest():
    import numpy as np
    bpy.context.view_layer.update()
    path=OUT/'docs/geometry_manifest.json'
    man=json.loads(path.read_text()) if path.exists() else {}
    man['links']={o.linkforge.link_name:{'world_matrix':[list(row) for row in o.matrix_world],'datum':o.get('datum','')} for o in scene.objects if o.linkforge.is_robot_link}
    man['meshes']=[]
    for o in scene.objects:
        if o.type!='MESH' or '_visual' not in o.name or not o.parent or not o.parent.linkforge.is_robot_link:continue
        evaluated=o.evaluated_get(bpy.context.evaluated_depsgraph_get())
        mesh=evaluated.to_mesh()
        try:
            vertices=np.empty(len(mesh.vertices)*3,dtype=np.float64)
            mesh.vertices.foreach_get('co',vertices);vertices=vertices.reshape(-1,3)
            t=np.array(evaluated.matrix_world);vertices=vertices@t[:3,:3].T+t[:3,3]
            man['meshes'].append({'name':o.name,'parent':o.parent.name,'faces':len(mesh.polygons),'bounds_m':[vertices.min(axis=0).tolist(),vertices.max(axis=0).tolist()]})
        finally:
            evaluated.to_mesh_clear()
    man['bounds_method']='Actual world-transformed evaluated mesh vertices; not rotated local bounding-box corners'
    path.write_text(json.dumps(man,indent=2)+'\n')
    return man
