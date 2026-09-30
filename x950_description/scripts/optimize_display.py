"""Reduce remaining dense display meshes for interactive ROS/Foxglove use.

Original source files stay unchanged. Each reduction uses the same 0.25 mm bounds
and 0.5 mm sampled surface limits as the avionics optimization. Small visuals are
left intact; a failure keeps the original mesh and is reported, not concealed.
"""
import bpy,json
from pathlib import Path
OUT=Path(bpy.data.filepath).resolve().parent
if OUT.name!='x950_description':OUT=OUT/'x950_description'
assert bpy.context.scene.name=='X950_ROS_LinkForge'
p=OUT/'scripts/optimize_avionics.py'
lib={'X950_OPTIMIZATION_LIBRARY_ONLY':True,'X950_PACKAGE_ROOT':str(OUT)}
exec(compile(p.read_text(),str(p),'exec'),lib)
plan={
 'e1r_sensor_visual':(.08,(12000,24000)),
 'd555_sensor_visual':(.08,(12000,24000)),
 'avia_sensor_visual':(.12,(5000,12000)),
 'x950_body_visual':(.15,(10000,20000)),
 'x950_avionics_bay_cover_visual':(.20,(2500,6000)),
 'x950_radio_antenna_port_visual':(.20,(2500,6000)),
 'x950_radio_antenna_starboard_visual':(.20,(2500,6000)),
 'gnss_main_bracket_visual':(.25,(3500,7000)),
 'gnss_aux_bracket_visual':(.25,(3500,7000)),
 'x950_landing_leg_starboard_visual':(.25,(2500,6000)),
 'x950_landing_leg_port_visual':(.25,(2500,6000)),
 'avia_bracket_visual':(.35,(2500,6500)),
 'x950_battery_bay_port_tab_visual':(.30,(2000,5000)),
 'x950_batterh_bay_starboard_tab_visual':(.30,(2000,5000)),
 'e1r_bracket_v01_visual':(.35,(2500,6500)),
}
for i in range(1,5):plan[f'x950_m{i}_motor_visual']=(.20,(6000,11000))
frames_before={o.name:tuple(v for row in o.matrix_world for v in row) for o in bpy.context.scene.objects if o.linkforge.is_robot_link}
reports=[]
for name,(ratio,target) in plan.items():
    try:
        r=lib['optimize_visual'](name,ratios=tuple(sorted(set((ratio,min(ratio*2,.75),.75)))),target_triangle_range=target,report_path=OUT/'docs'/f'{name}_mesh_optimization.json')
        a=r['attempts'][r['selected_attempt']]
        reports.append({'object':name,'status':'passed','before':r['original_triangle_count'],'after':r['optimized_triangle_count'],'bbox_max_error_m':max(max(row) for row in a['bbox_absolute_delta_m']),'sampled_surface_max_error_m':max(a['source_to_reduced']['maximum_m'],a['reduced_to_source']['maximum_m'])})
    except RuntimeError as e:
        reports.append({'object':name,'status':'original_retained','reason':str(e)})
    (OUT/'docs/display_mesh_optimization.json').write_text(json.dumps({'status':'in_progress','objects':reports},indent=2)+'\n')
assert frames_before=={o.name:tuple(v for row in o.matrix_world for v in row) for o in bpy.context.scene.objects if o.linkforge.is_robot_link}
triangles=0
for o in bpy.context.scene.objects:
    if o.type=='MESH' and '_visual' in o.name:
        o.data.calc_loop_triangles();triangles+=len(o.data.loop_triangles)
result={'status':'complete','total_display_triangles':triangles,'frames_unchanged':True,'source_files_unchanged':True,'sampled_validation_only':True,'objects':reports}
(OUT/'docs/display_mesh_optimization.json').write_text(json.dumps(result,indent=2)+'\n')
