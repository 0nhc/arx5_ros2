# arx5_ros

arx5机械臂的ROS 2功能包，纯Rviz可视化运动学效果，不依赖硬件，于Ubuntu22.04&ROS Humble测试通过。



# 依赖安装

```sh
sudo apt-get install liborocos-kdl-dev
```



# arx5_ros2安装

```sh
mkdir -p colcon_ws/src
cd colcon_ws/src
git clone https://github.com/purimagination/arx5_ros2
cd ..
colcon build
```



# Demo运行

```sh
# Terminal 1
ros2 launch arx5_bringup bringup.launch

# Terminal 2
ros2 topic pub /target_pose std_msgs/msg/Float64MultiArray "{data:[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" # x, y, z, roll, pitch, yaw
ros2 topic pub /target_pose std_msgs/msg/Float64MultiArray "{data:[0.2, 0.2, 0.2, 0.2, 0.2, 0.2]}" # x, y, z, roll, pitch, yaw
ros2 topic pub /target_pose std_msgs/msg/Float64MultiArray "{data:[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}" # x, y, z, roll, pitch, yaw
```
