# 3D target observation (fused camera + LiDAR)
std_msgs/Header header

geometry_msgs/Point position     # target center, header.frame_id is the frame
geometry_msgs/Vector3 velocity   # filtered velocity, same frame

uint8 METHOD_UNKNOWN=0
uint8 METHOD_LIDAR_POINTS=1      # LiDAR points inside the bbox
uint8 METHOD_LIDAR_GROUND=2      # camera ray x LiDAR ground plane (+ known radius)
uint8 METHOD_MONOCULAR=3         # apparent-size distance

uint8 method
float32 quality                  # 0..1
