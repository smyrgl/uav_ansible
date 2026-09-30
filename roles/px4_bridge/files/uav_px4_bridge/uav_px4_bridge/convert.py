"""uORB message contents -> ROS message field values. Pure functions, tested."""
import math

from .frames import (diag_covariance, frd_to_flu, ned_to_enu, px4_to_ros_orientation,
                     px4_to_ros_rotation, world_to_body)

EPOCH_FLOOR_US = 946_684_800_000_000   # 2000-01-01: below this a PX4 stamp is boot time, not epoch

POSE_FRAME_NED, POSE_FRAME_FRD = 1, 2
VELOCITY_FRAME_NED, VELOCITY_FRAME_FRD, VELOCITY_FRAME_BODY_FRD = 1, 2, 3

ARMING_STATE = {1: "disarmed", 2: "armed"}
NAV_STATE = {0: "manual", 1: "altitude", 2: "position", 3: "auto_mission", 4: "auto_loiter", 5: "auto_rtl",
             10: "acro", 12: "descend", 13: "termination", 14: "offboard", 15: "stabilized",
             17: "auto_takeoff", 18: "auto_land", 19: "auto_follow_target", 20: "auto_precland",
             21: "orbit", 22: "auto_vtol_takeoff"}
BATTERY_WARNING = {0: "none", 1: "low", 2: "critical", 3: "emergency", 4: "failed"}


def stamp_from_px4(timestamp_us, now_ns):
    """PX4 publishes epoch microseconds once XRCE timesync has run; before that
    (or without it) the field is boot time. Returns (stamp_ns, synchronized)."""
    ts = int(timestamp_us)
    if ts >= EPOCH_FLOOR_US:
        return ts * 1000, True
    return int(now_ns), False


def imu_fields(gyro_frd, accel_frd, attitude_q=None, orientation_variance_frd=None,
               gyro_stddev=0.01, accel_stddev=0.1):
    """sensor_msgs/Imu contents in FLU/ENU. Without an attitude the orientation
    covariance is [-1, ...] (unknown), per the message definition."""
    gyro = frd_to_flu(gyro_frd)
    accel = frd_to_flu(accel_frd)
    out = {"angular_velocity": gyro, "linear_acceleration": accel,
           "angular_velocity_covariance": diag_covariance([gyro_stddev ** 2] * 3),
           "linear_acceleration_covariance": diag_covariance([accel_stddev ** 2] * 3)}
    if attitude_q is None:
        out["orientation"] = (0.0, 0.0, 0.0, 1.0)
        out["orientation_covariance"] = [-1.0] + [0.0] * 8
    else:
        out["orientation"] = px4_to_ros_orientation(attitude_q)
        var = orientation_variance_frd if orientation_variance_frd is not None else [0.01] * 3
        out["orientation_covariance"] = diag_covariance([abs(float(v)) for v in var])
    return out


def odometry_fields(position_ned, q, velocity, velocity_frame, angular_velocity_frd,
                    position_variance, orientation_variance, velocity_variance, pose_frame=POSE_FRAME_NED):
    """nav_msgs/Odometry contents: pose in ENU (odom), twist in the FLU body frame."""
    if pose_frame != POSE_FRAME_NED:
        raise ValueError("unsupported pose frame %r" % pose_frame)
    r = px4_to_ros_rotation(q)
    pos = ned_to_enu(position_ned)
    if velocity_frame == VELOCITY_FRAME_NED:
        v_body = world_to_body(r, ned_to_enu(velocity))
        vel_var_order = (1, 0, 2)
    elif velocity_frame in (VELOCITY_FRAME_FRD, VELOCITY_FRAME_BODY_FRD):
        v_body = frd_to_flu(velocity)
        vel_var_order = (0, 1, 2)
    else:
        raise ValueError("unsupported velocity frame %r" % velocity_frame)
    return {"position": pos, "orientation": px4_to_ros_orientation(q),
            "linear": v_body, "angular": frd_to_flu(angular_velocity_frd),
            "pose_covariance": _pose_cov(diag_covariance(position_variance, (1, 0, 2)), diag_covariance(orientation_variance)),
            "twist_covariance": _pose_cov(diag_covariance(velocity_variance, vel_var_order), [0.0] * 9)}


def _pose_cov(pos3, rot3):
    cov = [0.0] * 36
    for i in range(3):
        for j in range(3):
            cov[i * 6 + j] = pos3[i * 3 + j]
            cov[(i + 3) * 6 + (j + 3)] = rot3[i * 3 + j]
    return cov


def local_position_fields(x, y, z, vx, vy, vz, xy_valid, z_valid, v_xy_valid, v_z_valid):
    """PoseWithCovariance / TwistWithCovariance contents in ENU (odom frame);
    invalid axes become NaN so no consumer mistakes them for zero."""
    pos = list(ned_to_enu((x, y, z)))
    vel = list(ned_to_enu((vx, vy, vz)))
    if not xy_valid:
        pos[0] = pos[1] = math.nan
    if not z_valid:
        pos[2] = math.nan
    if not v_xy_valid:
        vel[0] = vel[1] = math.nan
    if not v_z_valid:
        vel[2] = math.nan
    return {"position": tuple(pos), "linear": tuple(vel)}


def battery_fields(voltage_v, current_a, remaining, capacity_mah, discharged_mah, temperature,
                   cell_count, voltage_cell_v, connected, warning, time_remaining_s):
    """sensor_msgs/BatteryState contents. ROS current is negative when discharging;
    PX4's current_a is positive when discharging. Unknown -> NaN (or -1 where the
    ROS definition says so)."""
    def f(v, invalid):
        v = float(v)
        return math.nan if v == invalid or not math.isfinite(v) else v
    current = f(current_a, -1.0)
    cap_ah = math.nan if capacity_mah <= 0 else capacity_mah / 1000.0
    charge = math.nan if math.isnan(cap_ah) or discharged_mah < 0 else max(0.0, cap_ah - discharged_mah / 1000.0)
    pct = f(remaining, -1.0)
    if warning in (3, 4):
        status = 4   # POWER_SUPPLY_STATUS_NOT_CHARGING is 4? no: use FULL/DISCHARGING below
    status = 2 if (not math.isnan(current) and current > 0.0) else 0    # DISCHARGING / UNKNOWN
    health = {0: 1, 1: 1, 2: 1, 3: 1, 4: 5}.get(int(warning), 0)      # GOOD, or UNSPEC_FAILURE (5) when failed
    cells = [float(v) for v in voltage_cell_v[:max(0, int(cell_count))]]
    return {"voltage": f(voltage_v, 0.0), "current": (-current if not math.isnan(current) else math.nan),
            "charge": charge, "capacity": cap_ah, "design_capacity": cap_ah, "percentage": pct,
            "power_supply_status": status, "power_supply_health": health, "power_supply_technology": 3,  # LIPO
            "present": bool(connected), "cell_voltage": cells, "temperature": f(temperature, math.nan),
            "warning": BATTERY_WARNING.get(int(warning), str(warning)),
            "time_remaining_s": f(time_remaining_s, math.nan)}
