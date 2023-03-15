# Pose-Estimation-Aruco-ROS2

#Simulation
ros2 launch aruco_nav room_gazebo.launch.py

#Aruco detection
ros2 run ros2_aruco aruco_node --ros-args -p marker_size:=0.15

#1 run this for tf with respect to robot
# Prints out the Tag ID detected.
ros2 run aruco_nav fixed_static_transform.py

#2.1 run map navigation
//ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/aruco_ws/src/aruco_nav/maps/map.yaml use_sim_time:=True
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml use_sim_time:=True


#2.2 run this for position with respect to map
ros2 run aruco_nav tag_map_sub_node.py
