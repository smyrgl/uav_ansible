import binascii
import struct
import unittest

from uav_sensor_health.core import Assessment, ERROR, OK, WARN, cloud_header
from uav_sensor_health.health import Sample, check, d555_timing, grouped, lidar_health, px4_health, hflow_health, jetson_health, jetson_identity
from uav_sensor_health.observers import (sbf_blocks, chrony_assessment, ptp_assessment, gnss_health, parse_pvt_geodetic,
                                         parse_receiver_time, parse_receiver_status)


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

    def test_avia_validated_sensor_utc_makes_timing_good(self):
        driver = sample({'connected': '1', 'level': OK, 'message': 'Receiving point clouds'})
        cloud = sample({'points': 1000, 'frame_id': 'avia_link', 'layout_valid': True})
        clock = sample({'level': OK, 'message': 'Sensor UTC from PPS + pushed TOD', 'timing_validated': 'true',
                        'timestamp_type': '3', 'time_sync_status': '2', 'pps_status': '1'})
        result = lidar_health('avia', cloud, driver, clock, 20)
        self.assertEqual(result.level, OK)
        self.assertEqual(result.values['Timing/time_sync_status'], '2')

    def test_avia_clock_ok_level_without_validation_is_still_a_warning(self):
        driver = sample({'connected': '1', 'level': OK, 'message': 'Receiving point clouds'})
        cloud = sample({'points': 1000, 'frame_id': 'avia_link', 'layout_valid': True})
        clock = sample({'level': OK, 'message': 'Host receipt timestamps', 'timing_validated': 'false'})
        self.assertEqual(lidar_health('avia', cloud, driver, clock, 20).level, WARN)

    def test_d555_timing_verified_only_with_a_valid_model_and_utc_stamps(self):
        from uav_sensor_health.core import Assessment
        utc = Assessment(WARN, "Near host wall time; synchronization unverified",
                         {"classification": "epoch_compatible", "synchronization_verified": False})
        model = sample({'level': OK, 'message': 'D555 clock mapped to UTC', 'timing_validated': 'true',
                        'skew_ppm': '-11.8', 'residual_rms_us': '50.3'})
        good = d555_timing(utc, model, 20)
        self.assertEqual(good.level, OK)
        self.assertTrue(good.values['synchronization_verified'])
        invalid = sample({'level': WARN, 'message': 'D555 stamps on host receive time: collecting',
                          'timing_validated': 'false', 'reason': 'collecting: 3 of 10 one-second bins'})
        self.assertEqual(d555_timing(utc, invalid, 20).level, WARN)
        device = Assessment(WARN, "Device/non-epoch clock; UTC synchronization unverified",
                            {"classification": "device_clock"})
        self.assertEqual(d555_timing(device, model, 20).level, WARN)   # a valid model cannot vouch for device stamps
        self.assertEqual(d555_timing(utc, Sample(), 20).level, WARN)   # no model diagnostic at all

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
        self.assertEqual(result.level, OK)  # host receipt timing is accepted as the best available
        self.assertEqual(result.values['Timing/status'], 'OK')
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


def jetson_rows(now, tj="62.00C", tj_level=OK):
    return {
        "jetson_stats/board/Status": (now, OK, "NV Power[0] MAXN - JC inactive", {"NV Power-Mode": "MAXN", "jetson_clocks": "inactive"}, "aarch64"),
        "jetson_stats/board/Config": (now, OK, "NVIDIA Jetson Orin NX - Jetpack 7.2", {"Module": "NVIDIA Jetson Orin NX (16GB ram)", "Jetpack": "7.2"}, "aarch64"),
        "jetson_stats/temp/CPU": (now, OK, "52.50C", {}, "aarch64"),
        "jetson_stats/temp/tj": (now, tj_level, tj, {}, "aarch64"),
        "jetson_stats/temp/CV0": (now, OK, "Offline", {}, "aarch64"),
        "jetson_stats/power/VDD_IN": (now, OK, "18.4W", {"Power": "18.4W", "Average": "17.9W"}, "aarch64"),
        "jetson_stats/power/VDD_CPU_GPU_CV": (now, OK, "4200mW", {"Power": "4200mW"}, "aarch64"),
        "jetson_stats/cpu/0": (now, OK, " 41.00%", {}, "aarch64"),
        "jetson_stats/cpu/1": (now, OK, " 21.00%", {}, "aarch64"),
        "jetson_stats/cpu/2": (now, OK, "OFF", {}, "aarch64"),
        "jetson_stats/gpu/gpu": (now, OK, " 12.00%", {}, "aarch64"),
        "jetson_stats/fan/pwmfan": (now, OK, " 40%", {"Profile": "quiet", "RPM 0": "2100RPM"}, "aarch64"),
        "jetson_stats/mem/RAM": (now, OK, "21% - 9.8G lfb", {}, "aarch64"),
    }


class JetsonHealthTests(unittest.TestCase):
    def test_summary_row_from_jetson_stats(self):
        result = jetson_health(jetson_rows(100.0), 100.4)
        self.assertEqual(result.level, OK)
        self.assertEqual(result.message, "tj 62.0 C, VDD_IN 18.4 W, fan 40%, CPU 31 % x2, GPU 12 %, MAXN")
        self.assertEqual(result.values["Thermal/tj_c"], 62.0)
        self.assertEqual(result.values["Thermal/CPU_c"], 52.5)
        self.assertNotIn("Thermal/CV0_c", result.values)          # offline zones are skipped
        self.assertEqual(result.values["Power/VDD_CPU_GPU_CV"], "4200mW")
        self.assertEqual(result.values["Board/Module"], "NVIDIA Jetson Orin NX (16GB ram)")
        self.assertEqual(jetson_identity(jetson_rows(1.0)), "NVIDIA Jetson Orin NX (16GB ram)")

    def test_hottest_zone_without_tj_and_upstream_warn(self):
        rows = jetson_rows(50.0); del rows["jetson_stats/temp/tj"]
        rows["jetson_stats/temp/CPU"] = (50.0, WARN, "86.00C", {}, "aarch64")
        result = jetson_health(rows, 50.0)
        self.assertEqual(result.level, WARN)
        self.assertTrue(result.message.startswith("CPU 86.0 C"))
        self.assertIn("Thermal", result.values["Attention"])

    def test_bogus_upstream_threshold_does_not_alarm(self):
        rows = jetson_rows(10.0)
        rows["jetson_stats/temp/nvme Sensor 2"] = (10.0, ERROR, "65.85C more than -256.00C", {}, "aarch64")
        result = jetson_health(rows, 10.0)
        self.assertEqual(result.level, OK)
        self.assertAlmostEqual(result.values["Thermal/nvme Sensor 2_c"], 65.85, places=1)
        self.assertNotIn("CRITICAL", result.message)

    def test_critical_temperature_is_error_and_stale_rows_are_warn(self):
        rows = jetson_rows(100.0, tj="101.00C more than 100.00C", tj_level=ERROR)
        self.assertEqual(jetson_health(rows, 100.5).level, ERROR)
        self.assertIn("THERMAL CRITICAL", jetson_health(rows, 100.5).message)
        stale = jetson_health(rows, 120.0)
        self.assertEqual(stale.level, WARN)
        self.assertIn("stale for 20 s", stale.message)
        self.assertEqual(jetson_health({}, 1.0).level, WARN)
        self.assertEqual(jetson_identity({}), "Jetson")


class PX4BarometerTests(unittest.TestCase):
    def test_barometer_section_is_optional_then_informational(self):
        hb = {"HEARTBEAT": sample({"type": 2}, now=20)}
        without = px4_health(hb, {"connected": True, "message": "ok"}, 20, 0)
        self.assertNotIn("Barometer/status", without.values)
        with_baro = px4_health({**hb, "SCALED_PRESSURE": sample({"press_abs": 1013.25, "temperature": 2134, "time_boot_ms": 5}, now=20)},
                               {"connected": True, "message": "ok"}, 20, 0)
        self.assertEqual(with_baro.level, OK)
        self.assertEqual(with_baro.values["Barometer/message"], "FC baro 21.3 C, 1013.2 hPa")
        stale = px4_health({**hb, "SCALED_PRESSURE": sample({"press_abs": 1013.25, "temperature": 2134}, now=10)},
                           {"connected": True, "message": "ok"}, 20, 0)
        self.assertEqual(stale.values["Barometer/status"], "WARN")


def pvt_block(mode=4, error=0, sats=9, corr_age=120, h=12, v=20, reference_id=42):
    block = bytearray(96)
    block[:2] = b"$@"
    struct.pack_into("<HH", block, 4, 4007, 96)
    struct.pack_into("<IH", block, 8, 123000, 2380)
    block[14], block[15], block[74] = mode, error, sats
    struct.pack_into("<HH", block, 76, reference_id, corr_age)
    struct.pack_into("<HH", block, 90, h, v)
    struct.pack_into("<H", block, 2, binascii.crc_hqx(block[4:], 0))
    return bytes(block)


class GnssRtkTests(unittest.TestCase):
    timing = {"received": 20, "checks": {"PPS": check(True, "PPS locked")}}

    def test_parse_pvt_geodetic(self):
        values, stamp = parse_pvt_geodetic(pvt_block(mode=5, corr_age=123, h=36, v=50))
        self.assertEqual((values["mode"], values["mode_text"], values["satellites"]), (5, "RTK float", 9))
        self.assertEqual((values["mean_corr_age_sec"], values["h_accuracy_m"], values["v_accuracy_m"], values["reference_id"]),
                         (1.23, 0.36, 0.5, 42))
        self.assertEqual(stamp, (123000, 2380))
        self.assertIsNone(parse_pvt_geodetic(pvt_block(corr_age=65535))[0]["mean_corr_age_sec"])
        found, _ = sbf_blocks(pvt_block())
        self.assertEqual(found[0][0], 4007)

    def health(self, mode, require=True, **kw):
        values, _ = parse_pvt_geodetic(pvt_block(mode=mode, **kw))
        return gnss_health({"GNSS": sample({}), "GNSS_PVT": sample(values)}, self.timing, "broker", 20, require)

    def test_rtk_fixed_is_ok_and_anything_less_warns(self):
        fixed = self.health(4)
        self.assertEqual(fixed.level, OK)
        self.assertEqual(fixed.message, "Receiver data live; RTK fixed; PPS locked")
        self.assertEqual(fixed.values["Position/message"], "RTK fixed; 9 SVs; corrections 1.2 s old; H 0.12 m V 0.20 m")
        flt = self.health(5)
        self.assertEqual(flt.level, WARN)
        self.assertEqual(flt.message, "Receiver data live; RTK float (RTK fixed required); PPS locked")
        self.assertEqual(flt.values["Position/status"], "WARN")
        alone = self.health(1, corr_age=65535)
        self.assertEqual(alone.level, WARN)
        self.assertIn("stand-alone; 9 SVs; no corrections", alone.values["Position/message"])
        self.assertEqual(alone.values["Position/mean_corr_age_sec"], "n/a")
        self.assertEqual(self.health(5, require=False).level, OK)
        self.assertEqual(self.health(7).level, OK)   # moving-base RTK fixed counts as fixed


def receiver_time_block(sync=0x07, year=26, month=10, day=4, hour=1, minute=48, second=11, delta_ls=18):
    block = bytearray(24)
    block[:2] = b"$@"
    struct.pack_into("<HH", block, 4, 5914, 24)
    struct.pack_into("<IH", block, 8, 6491000, 2386)
    struct.pack_into("<bbbbbbbB", block, 14, year, month, day, hour, minute, second, delta_ls, sync)
    struct.pack_into("<H", block, 2, binascii.crc_hqx(block[4:], 0))
    return bytes(block)


def receiver_status_block(rx_state=0x00020070, rx_error=0, ext_error=0, cpu=14, uptime=903):
    block = bytearray(32)
    block[:2] = b"$@"
    struct.pack_into("<HH", block, 4, 4014, 32)
    struct.pack_into("<IH", block, 8, 6491000, 2386)
    struct.pack_into("<BBIII", block, 14, cpu, ext_error, uptime, rx_state, rx_error)
    struct.pack_into("<H", block, 2, binascii.crc_hqx(block[4:], 0))
    return bytes(block)


class GnssTimeTests(unittest.TestCase):
    """The receiver's FINETIME gates its PPS and the chrony feed (seen on
    2026-10-04: a cold start under an awning sat at WNSET+TOWSET for ten
    minutes while nothing pulsed); the row must say so."""
    timing = {"received": 20, "checks": {"PPS": check(False, "PPS not selected")}}

    def test_parse_receiver_time_and_status(self):
        values, stamp = parse_receiver_time(receiver_time_block(sync=0x03))
        self.assertEqual((values["sync_level"], values["sync_level_text"], values["wnset"], values["towset"], values["finetime"]),
                         ("0x03", "WNSET,TOWSET", True, True, False))
        self.assertEqual((values["receiver_utc"], values["leap_seconds"], stamp), ("2026-10-04 01:48:11", 18, (6491000, 2386)))
        self.assertIsNone(parse_receiver_time(receiver_time_block(year=-128, delta_ls=-128))[0]["receiver_utc"])
        status, _ = parse_receiver_status(receiver_status_block())
        self.assertEqual((status["rx_state"], status["rx_error"], status["ext_error"], status["finetime"], status["corrections_in"]),
                         ("WNSET,TOWSET,FINETIME,DIFFCORR_IN", "none", "none", True, True))
        self.assertEqual((status["cpu_load_pct"], status["uptime_s"]), (14, 903))
        self.assertEqual(parse_receiver_status(receiver_status_block(rx_error=1 << 5, ext_error=1))[0]["rx_error"], "ANTENNA")
        self.assertEqual([bid for bid, _ in sbf_blocks(receiver_time_block() + receiver_status_block())[0]], [5914, 4014])

    def health(self, sync, now=20, **status):
        samples = {"GNSS": sample({}), "GNSS_PVT": sample({"mode": 0, "error": 1}),
                   "GNSS_TIME": sample(parse_receiver_time(receiver_time_block(sync=sync))[0])}
        if status:
            samples["GNSS_STATUS"] = sample(parse_receiver_status(receiver_status_block(**status))[0])
        return gnss_health(samples, self.timing, "broker", now)

    def test_coarse_time_is_named_as_the_reason_for_no_pps(self):
        coarse = self.health(0x03)
        self.assertEqual(coarse.values["Receiver time/status"], "WARN")
        self.assertIn("Coarse time only (WNSET,TOWSET): no FINETIME yet", coarse.values["Receiver time/message"])
        self.assertEqual((coarse.values["Receiver time/finetime"], coarse.values["Receiver time/sync_level"]), (False, "0x03"))
        self.assertTrue(coarse.message.endswith("; coarse time"))
        fine = self.health(0x07)
        self.assertEqual(fine.values["Receiver time/status"], "OK")
        self.assertTrue(fine.message.endswith("; fine time"))
        self.assertEqual(fine.values["Receiver time/receiver_utc"], "2026-10-04 01:48:11")
        stale = self.health(0x07, now=30)
        self.assertEqual(stale.values["Receiver time/status"], "WARN")
        self.assertIn("No ReceiverTime block", stale.values["Receiver time/message"])

    def test_receiver_errors_are_flagged_and_absence_of_the_block_is_silent(self):
        clean = self.health(0x07, rx_state=0x00020070)
        self.assertEqual(clean.values["Receiver status/status"], "OK")
        self.assertEqual(clean.values["Receiver status/message"], "No receiver errors; CPU 14 %; up 903 s; state WNSET,TOWSET,FINETIME,DIFFCORR_IN")
        bad = self.health(0x07, rx_error=1 << 5)
        self.assertEqual(bad.values["Receiver status/status"], "WARN")
        self.assertEqual(bad.values["Receiver status/message"], "Receiver error flags: RxError ANTENNA, ExtError none")
        self.assertIn("Receiver status: Receiver error flags", bad.values["Attention"])
        self.assertNotIn("Receiver status/status", self.health(0x07).values)


class CloudHeaderTests(unittest.TestCase):
    def test_parses_a_hand_built_cdr_cloud(self):
        import struct
        def string(text):
            raw = text.encode() + b"\0"
            return struct.pack("<I", len(raw)) + raw
        def pad(buf):
            return buf + b"\0" * ((4 - (len(buf) - 4) % 4) % 4)
        buf = b"\x00\x01\x00\x00"                         # CDR_LE encapsulation
        buf += struct.pack("<iI", 1791136442, 157368411)       # stamp
        buf = pad(buf + string("avia_frame"))
        buf += struct.pack("<II", 1, 3)                        # height, width
        buf += struct.pack("<I", 2)                            # 2 fields
        for name, offset, datatype, count in (("x", 0, 7, 1), ("intensity", 4, 7, 1)):
            buf = pad(buf + string(name))
            buf += struct.pack("<I", offset) + struct.pack("<B", datatype)
            buf = pad(buf) + struct.pack("<I", count)
        buf += struct.pack("<B", 0)                            # is_bigendian
        buf = pad(buf) + struct.pack("<II", 8, 24)             # point_step, row_step
        buf += struct.pack("<I", 24) + b"\0" * 24                # data
        buf += struct.pack("<B", 1)                            # is_dense
        stamp, frame, height, width, point_step, row_step, nbytes = cloud_header(buf)
        self.assertEqual(stamp, 1791136442157368411)
        self.assertEqual((frame, height, width, point_step, row_step, nbytes), ("avia_frame", 1, 3, 8, 24, 24))
