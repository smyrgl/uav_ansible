#!/usr/bin/env python3
"""Validate expansion and pose overrides using the official ROS xacro package."""
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import xacro
ROOT=Path(__file__).resolve().parents[1]
NS='{http://www.ros.org/wiki/xacro}'

def signature(e):
    return (e.tag,tuple(sorted(e.attrib.items())),tuple(sorted((signature(c) for c in e),key=repr)))

def validate():
    path=ROOT/'urdf/x950.urdf.xacro'
    raw=ET.parse(path).getroot()
    urdf=ET.parse(ROOT/'urdf/x950.urdf').getroot()
    default=ET.fromstring(xacro.process_file(str(path)).toxml())
    assert not any(e.tag.startswith(NS) for e in default.iter())
    assert signature(default)==signature(urdf),'Default expansion differs from plain URDF'
    arguments={a.get('name'):a.get('default') for a in raw.findall(NS+'arg')}
    assert len(arguments)>=14 and len(arguments)%2==0
    for name,value in arguments.items():
        probe='0.011 -0.022 0.033'
        changed=ET.fromstring(xacro.process_file(str(path),mappings={name:probe}).toxml())
        expected=ET.fromstring(ET.tostring(default))
        found=0
        for j in raw.findall('joint'):
            o=j.find('origin')
            if o is None:continue
            for attr in ('xyz','rpy'):
                if o.get(attr)==f'$(arg {name})':
                    target=next(t for t in expected.findall('joint') if t.get('name')==j.get('name'))
                    target.find('origin').set(attr,probe);found+=1
        assert found==1
        assert signature(changed)==signature(expected),f'{name} override affected unexpected fields'
    report={'status':'passed','engine':'Official ROS xacro','default_matches_urdf':True,'pose_overrides_tested':len(arguments),'links':len(default.findall('link')),'joints':len(default.findall('joint')),'robot_state_publisher_or_rviz_tested':False}
    (ROOT/'docs/xacro_validation.json').write_text(json.dumps(report,indent=2)+'\n')
    return report

if __name__=='__main__':print(json.dumps(validate(),indent=2))
