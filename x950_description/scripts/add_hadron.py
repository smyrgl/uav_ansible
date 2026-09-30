"""Place drawing-derived Hadron 640R+ proxy low on front plate, thermal down."""
import bpy
from pathlib import Path
p=Path(bpy.data.filepath).resolve().parent
if p.name!='x950_description':p=p/'x950_description'
exec(compile((p/'scripts/scene_helpers.py').read_text(),str(p/'scripts/scene_helpers.py'),'exec'))
mount=link('hadron_mount_link',scene.objects['base_link'],(.151,0,.012),datum='Estimated rear mounting-plane center at lowest section of front center plate; user will refine placement. No bracket geometry supplied.')
mount['placement_status']='Provisional user-authorized estimate'
hadron=link('hadron_link',mount,relative_rpy=(math.pi,0,0),datum='Rear housing mounting-plane center; upright manufacturer proxy rolled pi so thermal is below visible. +X forward,+Y right,+Z down.')
hadron.linkforge.mass=.056;hadron['mass_kg']=.056;hadron['mass_status']='Manufacturer56g nominal; excludes bracket and cabling'
hadron['thermal_hfov_degrees']=32.;hadron['geometry_status']='Project proxy from manufacturer drawing; not official CAD'
T=Matrix.Diagonal((.001,.001,.001,1))
for part,color in [('body',(.25,.26,.27,1)),('thermal_lens',(.06,.065,.07,1)),('visible_lens',(.045,.05,.065,1))]:
    o=import_stl(OUT/'cad_source_meshes'/('hadron_640r_plus_proxy_'+part+'.stl'),hadron,T,material('hadron_'+part,color),'hadron_'+part+'_visual')
    o['geometry_status']='Drawing-derived proxy; see docs/HADRON_SOURCE.md'
for kind,xyz in [('thermal',(.04265,0,.0078)),('visible',(.04165,0,-.0122))]:
    o=link('hadron_nominal_'+kind+'_optical_frame',hadron,xyz,(-math.pi/2,0,-math.pi/2),datum='Illustrative lens-face reference, NOT calibrated optical center. Optical Z forward,X image-right,Y image-down; hardware is inverted.')
    o['calibration_status']='Nominal drawing proxy; optical entrance pupil unknown'
man=refresh_manifest()
result={'links':len(man['links']),'visual_meshes':len(man['meshes']),'thermal_below_visible':True,'pose_status':'estimate'}
