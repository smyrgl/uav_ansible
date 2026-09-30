"""Read-only checks of periodic PX4 1.17 uORB publications over DDS.

Event-only topics are intentionally not treated as periodic heartbeats. Topic
advertisements and an active XRCE agent never count as received sensor data.
"""
from .health import Sample, check, grouped, number

# Name, generated ROS type, conservative minimum delivery rate for bench health.
TOPICS = (
    ('vehicle_status', 'VehicleStatus', .5),
    ('vehicle_attitude', 'VehicleAttitude', 10.),
    ('vehicle_local_position', 'VehicleLocalPosition', 5.),
    ('vehicle_odometry', 'VehicleOdometry', 5.),
    ('sensor_combined', 'SensorCombined', 10.),
    ('timesync_status', 'TimesyncStatus', .5),
    ('vehicle_gps_position', 'SensorGps', .5),
    ('battery_status', 'BatteryStatus', .2),
)


def topic_name(name, message_type):
    version = getattr(message_type, 'MESSAGE_VERSION', 0)
    return '/fmu/out/' + name + (f'_v{version}' if version else '')


def dds_health(samples, topics, now, started, grace=10., *, middleware="unknown"):
    sections, live_count, healthy_count = {}, 0, 0
    for name, _, min_rate in TOPICS:
        sample = samples.get(name, Sample())
        live = sample.fresh(now, 3.)
        live_count += live
        metrics = sample.metrics(now)
        progressing = sample.advancing(now, 3.) and number(sample.values, 'source_timestamp_us', 0) > 0
        rate_ok = now - started < grace or metrics['observed_rate_hz'] >= min_rate
        good = live and progressing and rate_ok
        healthy_count += good
        reason = ('No current samples' if not live else 'Source timestamp not advancing' if not progressing
                  else 'Delivery rate low' if not rate_ok else 'Fresh samples; source timestamp advancing')
        sections[name] = check(good, reason, topic=topics[name], minimum_rate_hz=min_rate, **metrics)
    sync = samples.get('timesync_status', Sample())
    sync_ok = (sync.fresh(now, 3.) and number(sync.values, 'source_protocol') == 2
               and 0 <= number(sync.values, 'round_trip_time_us') < 50000
               and abs(number(sync.values, 'observed_offset_us') - number(sync.values, 'estimated_offset_us')) < 10000)
    sections['Timing'] = check(sync_ok, 'DDS timesync updates within 10 ms residual / 50 ms RTT' if sync_ok
                              else 'DDS timesync missing or outside bench tolerance',
                              **sync.metrics(now), sensor_hardware_sync_verified=False)
    connected = True if live_count else None if now - started < grace else False
    message = (f'{live_count}/{len(TOPICS)} monitored uORB topics live' if live_count
               else 'Waiting for PX4 DDS samples' if connected is None else 'No PX4 DDS samples received')
    if live_count and healthy_count < len(TOPICS): message += '; delivery degraded'
    elif live_count and not sync_ok: message += '; timing warning'
    result = grouped(connected, message, sections)
    result.values['Interface/firmware'] = 'PX4 1.17'
    result.values['Interface/middleware'] = middleware
    result.values['Interface/scope'] = 'Periodic topic delivery to this observer; not flight readiness or all ROS consumers'
    return result
