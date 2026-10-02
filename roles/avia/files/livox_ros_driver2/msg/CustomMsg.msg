# A Livox point frame (Livox's livox_ros_driver2 layout).
std_msgs/Header header  # stamp = timebase
uint64 timebase         # ns, time of the first point
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
CustomPoint[] points
