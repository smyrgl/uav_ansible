import binascii
import struct
import unittest

from uav_sensor_health.core import Assessment, ERROR, OK, WARN
from uav_sensor_health.health import Sample, check, grouped, lidar_health, px4_health, hflow_health
from uav_sensor_health.observers import sbf_blocks, chrony_assessment, ptp_assessment, gnss_health


def sample(values, now=20, stamp=None):
    s = Sample()
    s.observe(now, values, stamp)
    return s


class TestSensorHealth(unittest.TestCase):
    def test_only_disconnection_is_error(self):
        for raw_level in (OK, WARN, ERROR, 3):
            details = {'Timing': Assessment(raw_level, 'clock check', {})}
            self.assertNotEqual(grouped(True, 'connected', details).level, ERROR)
            self.assertEqual(grouped(False, 'disconnected', details).level, ERROR)
            self.assertEqual(grouped(None, 'unknown', details).level, WARN)

    def test_grouped_evidence_is_retained(self):
        result = grouped(True, 'data live', {'Data': check(True, 'fresh', rate_hz=20),
                                            'Timing': check(False, 'not synchronized')})
        self.assertEqual(result.level, WARN)
        self.assertEqual(result.values['Data/rate_hz'], 20)
        self.assertEqual(result.values['Timing/status'], 'WARN')

    def test_e1r_timing_failure_is_connected_warning(self):
        driver = sample({'msop_age_sec': '0.01', 'difop_age_sec': '0.02',
                         'level': ERROR, 'message': 'gPTP not synchronized'})
        result = lidar_health('e1r', Sample(), driver, Sample(), 20)
        self.assertEqual(result.level, WARN)
        self.assertEqual(result.values['Connection/state'], 'connected')
        self.assertIn('timing warning', result.message)

    def test_e1r_loss_of_all_packets_is_error_but_missing_monitor_unknown(self):
        driver = sample({'msop_age_sec': '10', 'difop_age_sec': '10', 'level': ERROR})
        self.assertEqual(lidar_health('e1r', Sample(), driver, Sample(), 20).level, ERROR)
        self.assertEqual(lidar_health('e1r', Sample(), driver, Sample(), 24).values['Connection/state'], 'unknown')

    def test_e1r_valid_cloud_and_timing_recovers(self):
        driver = sample({'msop_age_sec': '.01', 'difop_age_sec': '.01', 'level': OK,
                         'difop_time_mode': '3', 'difop_sync_status': '1',
                         'msop_time_mode': '3', 'utc_offset_verified': '1', 'message': 'Streaming'})
        cloud = sample({'points': 1000, 'frame_id': 'e1r_link', 'layout_valid': True})
        result = lidar_health('e1r', cloud, driver, Sample(), 20)
        self.assertEqual(result.level, OK)
        cloud.values['points'] = 0
        self.assertEqual(lidar_health('e1r', cloud, driver, Sample(), 20).level, WARN)

    def test_avia_connected_without_clouds_is_warning(self):
        driver = sample({'connected': '1', 'level': WARN})
        self.assertEqual(lidar_health('avia', Sample(), driver, Sample(), 20).level, WARN)
        driver.values['connected'] = '0'
        self.assertEqual(lidar_health('avia', Sample(), driver, Sample(), 20).level, ERROR)

    def test_avia_clock_diagnostic_metadata_does_not_collide_with_summary(self):
        driver = sample({'connected': '1', 'level': OK, 'message': 'Receiving point clouds'})
        clock = sample({'level': WARN, 'message': 'Host receipt timestamps', 'timestamp_type': '0'})
        cloud = sample({'points': 1000, 'frame_id': 'avia_link', 'layout_valid': True})
        result = lidar_health('avia', cloud, driver, clock, 20)
        self.assertEqual(result.level, WARN)
        self.assertEqual(result.values['Timing/timestamp_type'], '0')

    def test_connected_malformed_e1r_packets_are_warning(self):
        driver = sample({'connected': '1', 'msop_age_sec': '10', 'difop_age_sec': '10',
                         'level': WARN, 'message': 'Malformed E1R packet'})
        result = lidar_health('e1r', Sample(), driver, Sample(), 20)
        self.assertEqual(result.level, WARN)
        self.assertEqual(result.values['Connection/state'], 'connected')

    def test_router_alone_cannot_make_px4_healthy(self):
        transport = {'connected': True, 'message': 'TCP open'}
        self.assertEqual(px4_health({}, transport, 20, 0).level, ERROR)
        self.assertEqual(px4_health({}, transport, 5, 0).level, WARN)
        samples = {'HEARTBEAT': sample({'autopilot': 12})}
        self.assertEqual(px4_health(samples, transport, 20, 0).level, OK)
        self.assertEqual(px4_health(samples, transport, 24, 0).level, ERROR)

    def test_hflow_from_can_listener(self):
        flow = sample({'quality': 78, 'integration_timespan_us': 15872, 'pixel_flow_finite': True}, stamp=100)
        distance = sample({'range_m': 0.226, 'min_range': 0.08, 'max_range': 30.0}, stamp=100)
        driver = sample({'can_operstate': 'up', 'can_interface': 'can0', 'flow_hz': 72.0, 'range_hz': 49.7,
                         'hardware_id': 'H-Flow DroneCAN node 124'})
        result = hflow_health(flow, distance, driver, 20)
        self.assertEqual(result.values['Optical flow/status'], 'OK')
        self.assertEqual(result.values['Range/status'], 'OK')
        self.assertEqual(result.values['Bus/status'], 'OK')
        self.assertEqual(result.level, WARN)  # timing is host receipt, never verified
        flow.values['quality'] = 0
        self.assertEqual(hflow_health(flow, distance, driver, 20).values['Optical flow/status'], 'WARN')
        for s in (flow, distance):
            s.observe(30, s.values, 100)  # frozen stamps
        self.assertEqual(hflow_health(flow, distance, driver, 30).values['Connection/state'], 'unknown')
        self.assertEqual(hflow_health(Sample(), Sample(), Sample(), 20).level, WARN)  # nothing known yet

    def test_hflow_bus_down_is_a_disconnection(self):
        driver = sample({'can_operstate': 'down', 'can_interface': 'can0'})
        result = hflow_health(Sample(), Sample(), driver, 20)
        self.assertEqual(result.values['Connection/state'], 'disconnected')
        self.assertEqual(result.level, ERROR)

    def test_range_invalid_while_flow_lives_is_warning(self):
        flow = sample({'quality': 100, 'integration_timespan_us': 16000, 'pixel_flow_finite': True})
        driver = sample({'can_operstate': 'up'})
        for value in (float('nan'), float('inf'), 31.0, 0.01):
            distance = sample({'range_m': value, 'min_range': 0.08, 'max_range': 30.0})
            result = hflow_health(flow, distance, driver, 20)
            self.assertEqual(result.values['Range/status'], 'WARN')
            self.assertEqual(result.values['Connection/state'], 'connected')

    def test_clock_fault_never_invalidates_gnss_connection(self):
        sources = {'GNSS': sample({}), 'GNSS_PVT': sample({'mode': 4, 'error': 0})}
        timing = {'received': 20, 'checks': {'PPS': check(False, 'No PPS')}}
        result = gnss_health(sources, timing, 'broker connected', 20)
        self.assertEqual(result.level, WARN)
        self.assertEqual(result.values['Position/status'], 'OK')
        timing['checks']['PPS'] = check(True, 'PPS locked')
        self.assertEqual(gnss_health(sources, timing, 'broker connected', 20).level, OK)
        self.assertEqual(gnss_health(sources, timing, 'broker connected', 24).level, ERROR)

    def test_sample_memory_and_receipt_clock_are_bounded(self):
        s = Sample()
        for i in range(10000):
            s.observe(i / 100, {'n': i}, i)
        self.assertLessEqual(len(s.receipts), 512)
        self.assertTrue(s.fresh(100))
        self.assertFalse(s.fresh(105))
        self.assertEqual(s.metrics(105)['observed_rate_hz'], 0)


class TestClockEvidence(unittest.TestCase):
    def test_phc_requires_fresh_same_boot_verified_evidence(self):
        data = {'schema_version': 1, 'boot_id': 'abc', 'updated_monotonic_ns': 20_000_000_000,
                'utc_offset_verified': True, 'ptp_timescale': True}
        self.assertEqual(ptp_assessment(data, 'abc', 21_000_000_000).level, OK)
        for boot, now in (('def', 21_000_000_000), ('abc', 30_000_000_000), ('abc', 19_000_000_000)):
            self.assertEqual(ptp_assessment(data, boot, now).level, WARN)

    def test_chrony_pps_requires_current_reference_and_selected_source(self):
        tracking = ('Reference ID : 50505300 (PPS)\nLeap status : Normal\n'
                    'Ref time (UTC) : Sun Sep 27 17:35:30 2026\n'
                    'System time : 0.000000287 seconds slow of NTP time\nRoot dispersion : 0.000023707 seconds\n')
        wall = 1790530550
        self.assertEqual(chrony_assessment(tracking, '#* PPS 0 4 377 20', wall).level, OK)
        self.assertEqual(chrony_assessment(tracking, '#+ PPS 0 4 377 20', wall).level, WARN)
        self.assertEqual(chrony_assessment(tracking, '#* PPS', wall + 200).level, WARN)
        self.assertEqual(chrony_assessment('', '', wall).level, WARN)

    def test_sbf_split_corrupt_and_oversize_frames(self):
        block = bytearray(24)
        block[:2] = b'$@'
        struct.pack_into('<HH', block, 4, 5914, len(block))
        struct.pack_into('<H', block, 2, binascii.crc_hqx(block[4:], 0))
        found, tail = sbf_blocks(b'garbage' + block[:12])
        self.assertFalse(found)
        found, tail = sbf_blocks(tail + block[12:])
        self.assertEqual(found, [(5914, bytes(block))])
        corrupt = bytearray(block)
        corrupt[-1] ^= 1
        found, _ = sbf_blocks(corrupt + block)
        self.assertEqual(len(found), 1)
        corrupt = bytearray(block)
        struct.pack_into('<H', corrupt, 6, 65532)
        found, tail = sbf_blocks(corrupt + block)
        self.assertEqual(len(found), 1)
        self.assertLess(len(tail), 8192)


if __name__ == '__main__':
    unittest.main()
