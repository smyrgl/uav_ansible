import json,math,struct,time
import rclpy
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
rclpy.init();n=rclpy.create_node('verify_avia_bench')
clouds=[];diagnostics={};edges={};samples=[]
def cloud(m):
    fields={f.name:(f.offset,f.datatype) for f in m.fields}
    assert m.header.frame_id=='avia_nominal_lidar_frame'
    assert m.point_step==36 and m.row_step==m.width*36 and len(m.data)==m.row_step*m.height
    assert not m.is_bigendian
    assert fields['sensor_time_low']==(20,6) and fields['sensor_time_high']==(24,6)
    assert fields['point_offset_ns']==(28,6) and fields['x']==(0,7)
    assert m.width>0
    stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    clouds.append((time.monotonic(),stamp,m.width,time.time()-stamp))
    if len(samples)<3:
        x,y,z,i=struct.unpack_from('<ffff',m.data)
        assert all(math.isfinite(v) for v in (x,y,z,i)) and (x or y or z)
        low,high,off,status=struct.unpack_from('<IIII',m.data,20)
        samples.append(dict(x=x,y=y,z=z,intensity=i,timestamp_type=m.data[18],packet_time_raw=low+(high<<32),point_offset_ns=off,status=status))
def diag(m):
    for d in m.status:
        if d.name.startswith('avia/'):
            diagnostics[d.name]={'level':int.from_bytes(d.level) if isinstance(d.level,bytes) else d.level,'message':d.message,'values':{v.key:v.value for v in d.values}}
def tf(m):
    for t in m.transforms:edges[t.child_frame_id]={'parent':t.header.frame_id,'xyz':[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]}
q=QoSProfile(depth=2,reliability=ReliabilityPolicy.BEST_EFFORT)
subs=[n.create_subscription(PointCloud2,'/avia/points',cloud,q),n.create_subscription(DiagnosticArray,'/diagnostics',diag,10),n.create_subscription(TFMessage,'/tf_static',tf,QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL))]
end=time.monotonic()+18
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.2)
assert len(clouds)>=70,f'Only {len(clouds)} clouds'
hz=(len(clouds)-1)/(clouds[-1][0]-clouds[0][0]);assert 8<hz<12
assert all(b[1]>a[1] for a,b in zip(clouds,clouds[1:])), 'Host headers regressed'
assert max(c[3] for c in clouds)<1, 'Headers are not fresh host times'
chain=[];frame='avia_nominal_lidar_frame'
while frame!='base_link':
    assert frame in edges,f'No parent for {frame}'
    assert frame not in chain,'TF loop'
    chain.append(frame);frame=edges[frame]['parent']
chain.append(frame)
assert diagnostics['avia/driver']['level']==0,diagnostics
assert diagnostics['avia/clock']['level']==1
assert diagnostics['avia/clock']['values']['fusion_ready']=='false'
assert n.count_publishers('/avia/points')==1
print(json.dumps({'clouds':len(clouds),'rate_hz':hz,'point_count_range':[min(c[2] for c in clouds),max(c[2] for c in clouds)],'host_header_age_range_s':[min(c[3] for c in clouds),max(c[3] for c in clouds)],'samples':samples,'tf_chain':chain,'scan_frame':edges['avia_nominal_lidar_frame'],'diagnostics':diagnostics},indent=2),flush=True)
n.destroy_node();rclpy.shutdown()
