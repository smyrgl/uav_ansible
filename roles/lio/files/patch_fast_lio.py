#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
# The anchors and replacements below quote and modify FAST-LIO (hku-mars/FAST_LIO,
# GPL-2.0): this script is a patch to it and carries its licence.
"""uav_ansible's changes to FAST-LIO, applied to the build copy (the pinned checkout
stays pristine).

    patch_fast_lio.py <source dir>

Every edit replaces exact text; an anchor that is missing, or present a different
number of times than expected, stops the build: another FAST-LIO revision needs
these edits reviewed, not guessed.

1. C++17. Jazzy's rclcpp headers need it; FAST-LIO's CMakeLists forces C++14.
2. A gravity-aligned world frame. IMU_init takes gravity from the mean acceleration
   but leaves the attitude at identity, so camera_init is whatever attitude the IMU
   had at start-up: 45 deg of pitch for the Avia in its A-S+ cage, and still 5.4 deg
   on 2026-10-05 with the Avia level. The initial attitude is now the rotation that
   takes the measured specific force to +z: camera_init is level, its x along the
   IMU's heading at start-up, and gravity is (0, 0, -g).
3. Exit when lost, for systemd to restart it (Restart=always, no start limit).
   FAST-LIO does not come back by itself from a run of scans without effective
   points (its state runs away on the IMU alone) or from an impossible speed, and it
   keeps running and publishing meanwhile, so nothing else could tell systemd. Lost
   means no scan with effective points for uav.lost_exit_s of LiDAR time, or a speed
   above uav.max_speed_mps for uav.overspeed_s; the exit is announced on stderr with
   "FAST-LIO lost", which the lio watchdog reads as a divergence.
"""
import hashlib
import pathlib
import sys

CMAKE = [
    # (old, new, expected count; None = every occurrence, at least one)
    ("c++14", "c++17", None),
    ("CMAKE_CXX_STANDARD 14", "CMAKE_CXX_STANDARD 17", 1),
]

IMU = [(
    """  state_ikfom init_state = kf_state.get_x();
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);
""",
    """  state_ikfom init_state = kf_state.get_x();
  // uav_ansible: a gravity-aligned world frame (z up, x along the IMU's heading)
  init_state.grav = S2(V3D(0, 0, -G_m_s2));
  init_state.rot = SO3(Eigen::Quaterniond::FromTwoVectors(mean_acc.normalized(), V3D(0, 0, 1)));
""", 1)]

MAPPING = [
    ("#include <csignal>\n", "#include <csignal>\n#include <cstdlib>\n", 1),
    ("bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;\n",
     """bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;

// uav_ansible: exit when lost (no effective points for uav.lost_exit_s of LiDAR time,
// or faster than uav.max_speed_mps for uav.overspeed_s); systemd restarts FAST-LIO.
double uav_lost_exit_s = 3.0, uav_max_speed_mps = 30.0, uav_overspeed_s = 0.5;
double uav_last_tracked = -1.0, uav_overspeed_since = -1.0;
bool uav_effective = false;

void uav_lost(const char *why, double value)
{
    std::cerr << "FAST-LIO lost: " << why << " " << value << ", exiting for a restart" << std::endl;
    std::_Exit(3);
}

void uav_check_tracking(double t)
{
    if (uav_last_tracked < 0.0) uav_last_tracked = t;
    if (uav_lost_exit_s > 0.0 && t - uav_last_tracked > uav_lost_exit_s)
        uav_lost("no effective points for (s)", t - uav_last_tracked);
}
""", 1),
    ("""    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
""", """    if (effct_feat_num > 0) uav_effective = true;   // uav_ansible: this scan matched the map
    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
""", 1),
    ("""            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\\n");
                return;
""", """            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\\n");
                uav_check_tracking(lidar_end_time);
                return;
""", 1),
    ("""            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\\n");
                return;
""", """            if (feats_down_size < 5)
            {
                RCLCPP_WARN(this->get_logger(), "No point, skip this scan!\\n");
                uav_check_tracking(lidar_end_time);
                return;
""", 1),
    ("""            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
""", """            uav_effective = false;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            // uav_ansible: lost-track exit
            if (uav_effective) uav_last_tracked = lidar_end_time;
            uav_check_tracking(lidar_end_time);
            if (state_point.vel.norm() > uav_max_speed_mps)
            {
                if (uav_overspeed_since < 0.0) uav_overspeed_since = lidar_end_time;
                if (lidar_end_time - uav_overspeed_since >= uav_overspeed_s) uav_lost("speed (m/s)", state_point.vel.norm());
            }
            else uav_overspeed_since = -1.0;
""", 1),
    ("""        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
""", """        this->declare_parameter<vector<double>>("mapping.extrinsic_R", vector<double>());
        this->declare_parameter<double>("uav.lost_exit_s", 3.0);
        this->declare_parameter<double>("uav.max_speed_mps", 30.0);
        this->declare_parameter<double>("uav.overspeed_s", 0.5);
""", 1),
    ("""        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
""", """        this->get_parameter_or<vector<double>>("mapping.extrinsic_R", extrinR, vector<double>());
        this->get_parameter_or<double>("uav.lost_exit_s", uav_lost_exit_s, 3.0);
        this->get_parameter_or<double>("uav.max_speed_mps", uav_max_speed_mps, 30.0);
        this->get_parameter_or<double>("uav.overspeed_s", uav_overspeed_s, 0.5);
        if (extrinT.size() != 3 || extrinR.size() != 9)
        {
            RCLCPP_FATAL(this->get_logger(), "mapping.extrinsic_T/R missing (3 + 9 values): the Avia IMU to LiDAR transform");
            std::_Exit(2);
        }
""", 1),
]

FILES = {"CMakeLists.txt": CMAKE, "src/IMU_Processing.hpp": IMU, "src/laserMapping.cpp": MAPPING}


def apply(text, edits, name):
    for old, new, count in edits:
        found = text.count(old)
        if found == 0 or (count is not None and found != count):
            raise SystemExit(f"{name}: expected {count or 'some'} of {old.strip()[:70]!r}, found {found}")
        text = text.replace(old, new)
    return text


def fingerprint():
    """What the build tree's marker records: these edits, so a change to them re-prepares it."""
    return hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()


def main():
    if len(sys.argv) == 2 and sys.argv[1] == "--fingerprint":
        print(fingerprint())
        return
    if len(sys.argv) != 2:
        raise SystemExit(__doc__.split("\n\n")[1])
    root = pathlib.Path(sys.argv[1])
    for name, edits in FILES.items():
        path = root / name
        path.write_text(apply(path.read_text(), edits, name))
    (root / ".uav_patches").write_text(fingerprint() + "\n")
    print(f"patched {', '.join(FILES)} in {root}")


if __name__ == "__main__":
    main()
