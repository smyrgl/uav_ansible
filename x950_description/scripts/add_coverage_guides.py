"""Optional D555 coverage guides; D455-derived optical apex, not calibrated optics.

Guides are excluded from URDF and hidden initially. Enable the collection eye
in the Blender outliner to inspect them. Range is a drawing length, not a sensor
range specification. The guides follow the D555 nominal optical frame.
"""
import bpy
import math
from mathutils import Matrix
NAME='D555 coverage guides - nominal apex'
scene=bpy.context.scene
assert scene.name=='X950_ROS_LinkForge'
assert NAME not in bpy.data.collections,'Guides already exist'
collection=bpy.data.collections.new(NAME)
scene.collection.children.link(collection)
parent=None
length=.6
for name,h,v,color in [('Depth HD 16x9',87.,58.,(.15,.55,1.,1.)),('RGB',90.,65.,(1.,.35,.15,1.))]:
    a=length*math.tan(math.radians(h)/2);b=length*math.tan(math.radians(v)/2)
    corners=[(-a,-b,length),(a,-b,length),(a,b,length),(-a,b,length)]
    data=bpy.data.curves.new('D555 '+name+' field of view','CURVE')
    data.dimensions='3D';data.bevel_depth=.0008;data.resolution_u=1;data.bevel_resolution=0
    for points in [[(0,0,0),p] for p in corners]+[corners+[corners[0]]]:
        s=data.splines.new('POLY');s.points.add(len(points)-1)
        for target,p in zip(s.points,points):target.co=(*p,1)
    obj=bpy.data.objects.new('D555 '+name+' FOV nominal',data)
    collection.objects.link(obj)
    obj.parent=bpy.data.objects['d555_nominal_depth_optical_frame' if name.startswith('Depth') else 'd555_nominal_color_optical_frame'];obj.matrix_parent_inverse=Matrix.Identity(4);obj.matrix_basis=Matrix.Identity(4)
    obj.hide_render=True
    obj['horizontal_fov_deg']=h;obj['vertical_fov_deg']=v;obj['tolerance_deg']=3.
    obj['drawing_length_m']=length;obj['apex_status']='D455-derived optical centre; lateral CAD-confirmed, axial offset provisional.'
    mat=bpy.data.materials.new('D555 '+name+' guide color');mat.diffuse_color=color;data.materials.append(mat)
bpy.context.view_layer.update()
bpy.context.view_layer.layer_collection.children[NAME].hide_viewport=True
result={'coverage_guides':2,'hidden_by_default':True,'included_in_urdf':False}
