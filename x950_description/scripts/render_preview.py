"""Render review images from the generated Blender scene without changing geometry."""
import bpy
from pathlib import Path
from mathutils import Vector
OUT=Path(bpy.data.filepath).resolve().parent
if OUT.name!='x950_description': OUT=OUT/'x950_description'
scene=bpy.context.scene
assert scene.name=='X950_ROS_LinkForge'
scene.render.engine='BLENDER_WORKBENCH'
scene.render.resolution_x=1500
scene.render.resolution_y=1100
scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'
scene.render.film_transparent=False
shade=scene.display.shading
shade.light='STUDIO'
shade.studiolight_rotate_z=.4
shade.color_type='MATERIAL'
shade.show_shadows=True
shade.show_cavity=True
shade.cavity_type='BOTH'
shade.curvature_ridge_factor=1.3
shade.curvature_valley_factor=1.0
shade.show_specular_highlight=True
shade.background_type='WORLD'
scene.world=bpy.data.worlds.new('X950_Review_World')
scene.world.color=(.045,.055,.072)
camdata=bpy.data.cameras.new('X950_Review_Camera')
cam=bpy.data.objects.new('X950_Review_Camera',camdata)
scene.collection.objects.link(cam)
camdata.type='ORTHO'
camdata.lens=50
camdata.clip_start=.001
camdata.clip_end=100
scene.camera=cam
for name,location,target,scale in [
 ('x950_overview.png',(1.35,-1.65,.94),(0,0,-.015),1.52),
 ('x950_sensors.png',(.68,-.95,-.33),(.025,0,-.005),.65),
 ('x950_internals.png',(-.32,-.55,.42),(-.01,0,.045),.55),
]:
    cam.location=location
    cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler()
    camdata.ortho_scale=scale
    hidden=[]
    if name=='x950_sensors.png':
        for o in scene.objects:
            if o.type=='MESH' and o.name.startswith('x950_') and any(k in o.name for k in ('motor','arm','strut','landing_leg','antenna')):
                hidden.append((o,o.hide_render));o.hide_render=True
    if name=='x950_internals.png':
        for o in scene.objects:
            if o.type=='MESH' and (o.name.startswith('gnss_') or (o.name.startswith('x950_') and 'belly' not in o.name)):
                hidden.append((o,o.hide_render));o.hide_render=True
    scene.render.filepath=str(OUT/'docs'/name)
    bpy.ops.render.render(write_still=True)
    for o,was_hidden in hidden:o.hide_render=was_hidden
cam.hide_set(True)
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'x950_linkforge.blend'))
result={'previews':[str(OUT/'docs'/'x950_overview.png'),str(OUT/'docs'/'x950_sensors.png'),str(OUT/'docs'/'x950_internals.png')]}
