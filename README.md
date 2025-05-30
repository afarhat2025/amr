Notes on Launch Files:

amr_v4_base: (this launches diff drive controller, URDF, tf broadcaster and estop with GPIO comms and camera with LIDAR and kalman filtering)

ros2 launch amr_v4_base diffbot_system.launch.py

Mapping:

ros2 launch amr_v4_navigation amr_v4_mapping.launch.py localization:=False

Localization:

ros2 launch amr_v4_navigation amr_v4_mapping.launch.py localization:=True

Navigation:

ros2 launch amr_v4_navigation nav2_bringup.launch.py

To build package, please use below command for production release:

colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/AMRMAIN
. install/setup.bash

For rtabmap and rtabmap-ros use (adjust path of Torch library)
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_TORCH=ON -DWITH_PYTHON=ON -DTorch_DIR=/home/amr/.local/lib/python3.8/site-packages/torch/share/cmake/Torch --packages-up-to rtabmap_ros


Building sick lidar, use below
colcon build --packages-select sick_scan_xd --cmake-args " -DCMAKE_BUILD_TYPE=Release -DROS_VERSION=2" --event-handlers console_direct+

"# amr" 
