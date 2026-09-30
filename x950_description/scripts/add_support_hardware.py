"""Add official Holybro assembly and replace both factory GNSS assemblies.
See docs/HOLYBRO_SOURCE.md and ANTENNA_NOTES.md for provenance and datums.
"""
import bpy
from pathlib import Path
p=Path(bpy.data.filepath).resolve().parent
if p.name!='x950_description':p=p/'x950_description'
exec(compile((p/'scripts/scene_helpers.py').read_text(),str(p/'scripts/scene_helpers.py'),'exec'))
base=scene.objects['base_link']
board=link('avionics_carrier_link',base,(.065,0,.075),datum='Estimated front-bay placement: footprint center at existing standoff feet; native +X provisionally aircraft forward. NOT measured FC/IMU datum.')
board['placement_status']='User-authorized estimate; caliper later';board['hardware_configuration']='Official Holybro Jetson/Pixhawk6X assembly; exact installed Orin variant unconfirmed'
boardmat=material('avionics_assembly_provisional',(.12,.15,.14,1))
T=Matrix(((.001,0,0,-.063),(0,.001,0,-.040),(0,0,.001,.0080254),(0,0,0,1)))
import_stl(OUT/'cad_source_meshes/holybro_jetson_baseboard.stl',board,T,boardmat,'avionics_carrier_visual')
fmu=link('fmu_housing_link',board,(.0206,0,.01832272),datum='Geometric center of Pixhawk 6X CAD housing. Axes inherited from provisional baseboard placement; NOT an IMU or calibrated flight-controller frame.')
fmu['is_imu_datum']=False
mountmat=material('gnss_PAHT_CF',(.035,.035,.035,1))
radomemat=material('gnss_radome_provisional',(.68,.69,.7,1))
for label,old,xyz,yaw in [('main','x950_gnss_antenna_main_visual',(-.334278190058,.336696488345,.092500000245),math.radians(172)),('aux','x950_gnss_antenna_alt_visual',(-.334278208045,-.336696472709,.092500000245),-math.pi/2)]:
    assert old in scene.objects
    bpy.data.objects.remove(scene.objects[old],do_unlink=True)
    mount=link('gnss_'+label+'_mount_link',base,xyz,(0,0,yaw),datum='v02 bracket build origin at foot-pad plane, fitted to factory aircraft arm interface. Main=port, aux=starboard mechanical labels only.')
    T=Matrix(((.001,0,0,0),(0,-.001,0,0),(0,0,-.001,.03048),(0,0,0,1)))
    import_stl(ROOT/'Antenna/rtk_antenna_mount_v02.stl',mount,T,mountmat,'gnss_'+label+'_bracket_visual')
    ant=link('gnss_'+label+'_link',mount,(0,0,.03048),datum='MAN1216Q50 mounting-seat center; +Z up. Mechanical datum, NOT antenna phase center.')
    ant.linkforge.mass=.025;ant['mass_kg']=.025;ant['mass_status']='Manufacturer nominal25g; mounting hardware excluded'
    # Drawing supplies overall envelope only. Undimensioned shoulder/taper is illustrative.
    rings=[(0,.025),(.006,.025),(.009,.024),(.0395,.0205),(.04216,.0185)]
    n=96;verts=[(r*math.cos(2*math.pi*i/n),r*math.sin(2*math.pi*i/n),z) for z,r in rings for i in range(n)]
    faces=[]
    for k in range(len(rings)-1):
        for i in range(n):
            j=(i+1)%n;faces.append((k*n+i,k*n+j,(k+1)*n+j,(k+1)*n+i))
    faces += [tuple(reversed(range(n))),tuple((len(rings)-1)*n+i for i in range(n))]
    mesh=bpy.data.meshes.new('MAN1216Q50_envelope');mesh.from_pydata(verts,[],faces);mesh.update()
    o=bpy.data.objects.new('gnss_'+label+'_radome_visual',mesh);scene.collection.objects.link(o)
    adopt_mesh(o,ant,Matrix.Identity(4),radomemat,o.name)
    o['geometry_status']='Manufacturer50mmmaxdiameter×42.16mmheight; shoulder/taper estimated. Connector recess and leads omitted.'
man=refresh_manifest()
result={'links':len(man['links']),'visual_meshes':len(man['meshes']),'avionics_placement':'estimate','antenna_placement':'source bracket fit'}
