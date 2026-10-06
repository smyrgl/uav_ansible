#!/usr/bin/env python3
"""Check each named visual's world bounds against Blender's saved manifest.

Requires NumPy. This is geometry/static validation, not a ROS runtime test.
"""
from pathlib import Path
import json
import math
import struct
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def pose(xyz='0 0 0',rpy='0 0 0'):
    x,y,z=map(float,xyz.split());r,p,w=map(float,rpy.split())
    sr,cr,sp,cp,sw,cw=math.sin(r),math.cos(r),math.sin(p),math.cos(p),math.sin(w),math.cos(w)
    t=np.eye(4)
    t[:3,:3]=[[cw*cp,cw*sp*sr-sw*cr,cw*sp*cr+sw*sr],[sw*cp,sw*sp*sr+cw*cr,sw*sp*cr-cw*sr],[-sp,cp*sr,cp*cr]]
    t[:3,3]=x,y,z
    return t

def origin(element):
    o=element.find('origin')
    return pose() if o is None else pose(o.get('xyz','0 0 0'),o.get('rpy','0 0 0'))

def validate():
    xml=ET.parse(ROOT/'urdf/x950.urdf').getroot()
    manifest=json.loads((ROOT/'docs/geometry_manifest.json').read_text())
    links={e.get('name'):e for e in xml.findall('link')}
    assert len(links)==len(xml.findall('link'))==len(manifest['links'])
    joints=xml.findall('joint')
    assert len(joints)==len(links)-1 and all(j.get('type')=='fixed' for j in joints)
    parents={j.find('child').get('link'):j for j in joints}
    assert len(parents)==len(joints)
    assert set(links)-set(parents)=={'base_link'}
    world={'base_link':np.eye(4)}
    def world_pose(name,visiting=None):
        if name in world:return world[name]
        visiting=set() if visiting is None else visiting
        assert name not in visiting,'TF cycle'
        visiting.add(name)
        j=parents[name];parent=j.find('parent').get('link')
        assert parent in links
        world[name]=world_pose(parent,visiting)@origin(j)
        return world[name]
    frame_errors={}
    for name,details in manifest['links'].items():
        t=world_pose(name)
        delta=float(np.max(np.abs(t-np.array(details['world_matrix']))))
        assert delta<2e-6,(name,delta)
        assert abs(np.linalg.det(t[:3,:3])-1)<1e-9
        frame_errors[name]=delta
    source_bounds={};source_visuals={}
    for mesh in manifest['meshes']:
        visual_name=mesh['name']
        assert visual_name not in source_visuals,('Duplicate source visual',visual_name)
        source_visuals[visual_name]=mesh
        b=np.array(mesh['bounds_m']);name=mesh['parent']
        assert name in links,('Source visual has unknown parent',visual_name,name)
        assert b.shape==(2,3) and np.isfinite(b).all(),('Invalid source bounds',visual_name)
        if name not in source_bounds:source_bounds[name]=b
        else:source_bounds[name]=np.stack((np.minimum(source_bounds[name][0],b[0]),np.maximum(source_bounds[name][1],b[1])))
    exported_bounds={};exported_visual_bounds={};triangles=0;files=[];visual_errors={};visual_files={}
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attr','<u2')])
    for name,link in links.items():
        assert not link.findall('inertial') and not link.findall('collision')
        for visual in link.findall('visual'):
            visual_name=visual.get('name')
            assert visual_name in source_visuals,(
                'Visual must name its Blender source; re-export with source_name metadata',
                name,visual_name)
            assert visual_name not in visual_errors,('Duplicate exported visual',visual_name)
            source=source_visuals[visual_name]
            assert source['parent']==name,('Visual changed parent',visual_name,source['parent'],name)
            mesh=visual.find('geometry/mesh');assert mesh is not None
            prefix='package://x950_description/'
            uri=mesh.get('filename');assert uri.startswith(prefix)
            path=ROOT/uri.removeprefix(prefix)
            assert path.is_file()
            with path.open('rb') as f:
                header=f.read(84)
                assert len(header)==84,('Truncated binary STL',path)
                count=struct.unpack('<I',header[80:84])[0]
                data=np.fromfile(f,dtype=dtype,count=count)
            assert len(data)==count and count>0 and path.stat().st_size==84+50*count
            v=data['vertices'].reshape(-1,3).astype(float)
            assert np.isfinite(v).all()
            scale=np.array(list(map(float,mesh.get('scale','1 1 1').split())))
            assert np.allclose(scale,1),'Mesh scale not baked'
            t=world_pose(name)@origin(visual)
            v=v@t[:3,:3].T+t[:3,3]
            b=np.stack((v.min(axis=0),v.max(axis=0)))
            visual_error=float(np.max(np.abs(b-np.array(source['bounds_m']))))
            assert visual_error<3e-6,(
                visual_name,'individual visual placement error metres',visual_error)
            visual_errors[visual_name]=visual_error
            visual_files[visual_name]=str(path.relative_to(ROOT))
            exported_visual_bounds[visual_name]=b
            if name not in exported_bounds:exported_bounds[name]=b
            else:exported_bounds[name]=np.stack((np.minimum(exported_bounds[name][0],b[0]),np.maximum(exported_bounds[name][1],b[1])))
            triangles+=count;files.append(str(path.relative_to(ROOT)))
    assert set(visual_errors)==set(source_visuals),(
        'Missing exported visuals',sorted(set(source_visuals)-set(visual_errors)))
    assert len(files)==len(set(files)), 'Different visuals unexpectedly share a mesh file'
    errors={}
    for name,b in source_bounds.items():
        error=float(np.max(np.abs(b-exported_bounds[name])))
        assert error<3e-6,(name,'mesh placement error metres',error)
        errors[name]=error
    # Independent installation-direction checks catch sign/optical convention mistakes.
    # A-S+ nose set (2026-10-05, Printables/Pitch_Study/build/urdf_poses.json): the D555
    # is inverted and 40 deg nose-down, the Avia 45 deg nose-down.
    down40=np.array([math.cos(0.698132),0,-math.sin(0.698132)])
    down45=np.array([math.cos(0.785398),0,-math.sin(0.785398)])
    assert np.allclose(world['d555_link'][:3,:3]@np.array([1,0,0]),down40,atol=1e-6)
    assert world['d555_link'][2,2]<0,'D555 must be inverted (its Z points down)'
    assert np.allclose(world['d555_nominal_depth_optical_frame'][:3,:3]@np.array([0,0,1]),down40,atol=2e-6)
    assert np.allclose(world['e1r_nominal_lidar_frame'][:3,:3]@np.array([1,0,0]),[0,0,-1],atol=1e-6)
    assert np.allclose(world['avia_link'][:3,3],[.183367,0,.053776],atol=1e-6)
    assert np.allclose(world['avia_link'][:3,:3]@np.array([1,0,0]),down45,atol=1e-6)
    assert np.allclose(world['d555_link'][:3,3],[.143367,0,-.094827],atol=1e-6)
    if 'hadron_nominal_thermal_optical_frame' in links:
        assert np.allclose(world['hadron_nominal_thermal_optical_frame'][:3,3],[.209855,0,-.003504],atol=2e-6)
    if 'battery_link' in links:
        assert np.allclose(world['battery_link'][:3,3],[-.02623,0,.0341],atol=1e-6)
        assert np.allclose(exported_bounds['battery_link'][1]-exported_bounds['battery_link'][0],[.1915,.1036,.0726],atol=2e-6)
    optional_checks={}
    gnss_frames={'gnss_main_link','gnss_aux_link','gnss_main_mount_link','gnss_aux_mount_link'}
    if gnss_frames & set(links):
        assert gnss_frames <= set(links),('Incomplete two-antenna assembly',gnss_frames-set(links))
        for label in ('main','aux'):
            antenna=world[f'gnss_{label}_link'];mount=world[f'gnss_{label}_mount_link']
            assert np.allclose(antenna[:3,:3]@np.array([0,0,1]),[0,0,1],atol=2e-6),('GNSS must point up',label)
            assert np.allclose(antenna[:3,3]-mount[:3,3],[0,0,.03048],atol=2e-6),('GNSS seat height relative to bracket',label)
            assert abs(mount[2,3]-.092500000245)<2e-6,('GNSS bracket foot height',label)
            assert abs(antenna[2,3]-.122980000245)<2e-6,('GNSS seat height in aircraft',label)
            antenna_bounds=exported_bounds[f'gnss_{label}_link']
            assert abs(antenna_bounds[0,2]-antenna[2,3])<3e-6,('Radome must start at mounting seat',label)
            assert abs(antenna_bounds[1,2]-antenna_bounds[0,2]-.04216)<3e-6,('Radome height',label)
        baseline=world['gnss_main_link'][:3,3]-world['gnss_aux_link'][:3,3]
        assert np.allclose(baseline,[0,.673392961,0],atol=2e-6),('Mechanical antenna seat baseline',baseline)
        assert world['gnss_main_link'][1,3]>0 and world['gnss_aux_link'][1,3]<0,'Main must be port and aux starboard'
        assert not {'x950_gnss_antenna_main_visual','x950_gnss_antenna_alt_visual'} & set(source_visuals),'Factory GNSS assemblies still present'
        optional_checks['gnss']='Two upright radomes, bracket/seat heights, radome heights, mechanical baseline and factory replacement checked; phase centres not asserted'
    hadron_frames={'hadron_link','hadron_nominal_thermal_optical_frame','hadron_nominal_visible_optical_frame'}
    if hadron_frames & set(links):
        assert hadron_frames <= set(links),('Incomplete Hadron frame set',hadron_frames-set(links))
        # A-S+ nose set 2026-10-05 (base v2 + carrier v1, 15 deg variant): upright housing (roll 0), 15 deg nose-down about the rear housing face centre at (0.166639, 0, 0).
        pitch=np.radians(15.0)
        housing=world['hadron_link'][:3,:3]
        assert np.allclose(housing@np.array([0,0,1]),[np.sin(pitch),0,np.cos(pitch)],atol=2e-6),'Hadron housing must be upright and pitched 15 deg nose-down'
        assert np.allclose(housing@np.array([0,1,0]),[0,1,0],atol=2e-6),'Hadron housing must have zero roll and yaw'
        assert np.allclose(world['hadron_mount_link'][:3,3],[.166639,0,0],atol=2e-6),'Hadron rear housing face centre must be at x 0.166639, z 0 (base v2 + carrier v1, 15 deg)'
        assert np.allclose(world['hadron_link'][:3,3],world['hadron_mount_link'][:3,3],atol=2e-6),'Hadron link must sit on its mount'
        for frame in hadron_frames-{'hadron_link'}:
            assert np.allclose(world[frame][:3,:3]@np.array([0,0,1]),housing@np.array([1,0,0]),atol=2e-6),(frame,'Optical Z must point along the housing forward axis')
        assert world['hadron_nominal_thermal_optical_frame'][2,3]>world['hadron_nominal_visible_optical_frame'][2,3],'Hadron thermal must be above visible (upright)'
        assert 'hadron_link' in exported_bounds,'Hadron visual geometry missing'
        for visual_name in ('hadron_thermal_lens_visual','hadron_visible_lens_visual','hadron_plate_v2_visual','hadron_base_v2_visual','hadron_carrier_v1_15deg_visual','hadron_backing_block_v2_lower_visual','hadron_backing_block_v2_upper_visual'):
            assert visual_name in exported_visual_bounds,('Hadron visual missing',visual_name)
        thermal_center=np.mean(exported_visual_bounds['hadron_thermal_lens_visual'],axis=0)
        visible_center=np.mean(exported_visual_bounds['hadron_visible_lens_visual'],axis=0)
        assert thermal_center[2]>visible_center[2],'Hadron thermal mesh must be above visible mesh'
        optional_checks['hadron']='Upright housing pitched 15 deg nose-down on base v2 + carrier v1 (A-S+ 2026-10-05), thermal frame and mesh above visible, both nominal optical axes along the housing axis, and plate v2 / base v2 / carrier v1 / backing block v2 visuals checked'
    report={'status':'passed','scope':'Static TF and each named visual world-space bounding box compared to Blender scene; not full surface equivalence','links':len(links),'fixed_joints':len(joints),'visual_meshes':len(files),'triangles':triangles,'max_frame_matrix_error':max(frame_errors.values()),'mesh_bounds_errors_m':errors,'per_visual_bounds_errors_m':visual_errors,'per_visual_mesh_files':visual_files,'max_visual_bounds_error_m':max(visual_errors.values(),default=0.0),'sensor_viewing_directions':'passed','additional_installation_checks':optional_checks,'ros_distro_target':'jazzy','ros_runtime_tested':False}
    (ROOT/'docs/geometry_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':print(json.dumps(validate(),indent=2))
