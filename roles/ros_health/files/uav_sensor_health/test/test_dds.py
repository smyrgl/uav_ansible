import unittest
from uav_sensor_health.dds import TOPICS, topic_name, dds_health
from uav_sensor_health.health import Sample
from uav_sensor_health.core import OK, WARN, ERROR

class DDSHealthTest(unittest.TestCase):
    def setUp(self):
        self.topics={name:'/fmu/out/'+name for name,_,_ in TOPICS}
    def live(self):
        samples={}
        for name,_,_ in TOPICS:
            s=samples[name]=Sample()
            for i in range(100):
                values=dict(source_timestamp_us=100000+i,source_protocol=2,
                            round_trip_time_us=1000,observed_offset_us=900,estimated_offset_us=1000)
                s.observe(6+i*.04,values,100000+i)
        return samples
    def test_discovery_without_samples_is_error_after_grace(self):
        self.assertEqual(dds_health({},self.topics,20,0).level,ERROR)
        self.assertEqual(dds_health({},self.topics,1,0).level,WARN)
    def test_live_topics_are_ok(self):
        self.assertEqual(dds_health(self.live(),self.topics,10,0).level,OK)
    def test_one_missing_is_warning_not_disconnected(self):
        s=self.live();del s['vehicle_attitude']
        self.assertEqual(dds_health(s,self.topics,10,0).level,WARN)
    def test_all_stale_are_error(self):
        self.assertEqual(dds_health(self.live(),self.topics,20,0).level,ERROR)
    def test_frozen_timestamp_is_warning(self):
        s=self.live();s['vehicle_attitude'].progress=1
        self.assertEqual(dds_health(s,self.topics,10,0).level,WARN)
    def test_timesync_warning_does_not_disconnect(self):
        s=self.live();s['timesync_status'].values['observed_offset_us']=10000000
        a=dds_health(s,self.topics,10,0)
        self.assertEqual(a.level,WARN);self.assertEqual(a.values['Connection/state'],'connected')
    def test_rate_degradation_and_recovery(self):
        s=self.live();s['vehicle_attitude'].receipts.clear()
        self.assertEqual(dds_health(s,self.topics,11,0).level,WARN)
        self.assertEqual(dds_health(self.live(),self.topics,10,0).level,OK)
    def test_versioned_topic_names(self):
        class V: MESSAGE_VERSION=1
        self.assertEqual(topic_name('vehicle_status',V),'/fmu/out/vehicle_status_v1')
        self.assertEqual(topic_name('sensor_combined',object),'/fmu/out/sensor_combined')
