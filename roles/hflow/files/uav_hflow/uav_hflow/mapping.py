"""DroneCAN H-Flow measurements -> PX4-typed fields. Pure functions, no ROS.

The flow message (com.hex.equipment.flow.Measurement, 20200) carries what PX4's
own uavcan flow driver turns into uORB sensor_optical_flow; the range message
(uavcan.equipment.range_sensor.Measurement, 1050) becomes distance_sensor. The
same mapping here keeps the Jetson-side topics comparable with /fmu/out.
"""
import math
from dataclasses import dataclass

READING_UNDEFINED, READING_VALID, READING_TOO_CLOSE, READING_TOO_FAR = 0, 1, 2, 3
SENSOR_TYPE_LIDAR = 2
DEVICE_BUS_TYPE_UAVCAN = 3          # PX4 device::Device::DeviceBusType_UAVCAN


@dataclass(frozen=True)
class RangeSample:
    range_m: float
    reading_type: int
    field_of_view_rad: float
    sensor_type: int
    sensor_id: int


def px4_device_id(node_id, bus=0, devtype=0):
    """PX4 DeviceId layout: bus_type[0:3] bus[3:8] address[8:16] devtype[16:24]."""
    return (DEVICE_BUS_TYPE_UAVCAN & 0x7) | ((bus & 0x1F) << 3) | ((node_id & 0xFF) << 8) | ((devtype & 0xFF) << 16)


def range_fields(sample, min_range, max_range):
    """Return (sensor_msgs/Range.range, DistanceSensor.current_distance, signal_quality).

    ROS Range: -inf below the minimum, +inf beyond the maximum, NaN unknown.
    PX4 DistanceSensor: the sensor's own limit when clipped, quality 0 when
    the reading is not a valid distance, -1 (unknown quality) when it is.
    """
    if sample.reading_type == READING_VALID and math.isfinite(sample.range_m):
        return sample.range_m, sample.range_m, -1
    if sample.reading_type == READING_TOO_CLOSE:
        return -math.inf, min_range, 0
    if sample.reading_type == READING_TOO_FAR:
        return math.inf, max_range, 0
    return math.nan, math.nan, 0


def flow_fields(integration_interval_s, gyro_integral, flow_integral, quality,
                distance_m=None, distance_age_s=None, distance_max_age_s=0.5):
    """SensorOpticalFlow field values from one flow measurement.

    pixel_flow and delta_angle stay in the sensor's own (FRD) axes, as PX4
    publishes them. The distance is the latest valid range if it is fresh.
    """
    have_distance = (distance_m is not None and distance_age_s is not None
                     and math.isfinite(distance_m) and 0 <= distance_age_s <= distance_max_age_s)
    return {
        "pixel_flow": [float(flow_integral[0]), float(flow_integral[1])],
        "delta_angle": [float(gyro_integral[0]), float(gyro_integral[1]), math.nan],
        "delta_angle_available": True,
        "distance_m": float(distance_m) if have_distance else math.nan,
        "distance_available": have_distance,
        "integration_timespan_us": max(0, int(round(float(integration_interval_s) * 1e6))),
        "quality": max(0, min(255, int(quality))),
    }


def stream_rate_hz(receipts, now, window_s=5.0):
    recent = [t for t in receipts if t >= now - window_s]
    if len(recent) < 2:
        return 0.0
    span = recent[-1] - recent[0]
    return (len(recent) - 1) / span if span > 0 else 0.0
