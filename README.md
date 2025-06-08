# Sim-env rover navigation
## Overview
In this package, odometry is calculated from the number of tire rotations and steering rotation angle. 

![Screenshot from 2025-04-20 14-57-00](https://github.com/user-attachments/assets/e0b11be1-71d2-4f70-abc7-6df95a424d7e)

![Screenshot from 2025-04-20 14-56-52](https://github.com/user-attachments/assets/1c3208e9-d178-4de8-9aa2-e97f7e73a7e1)



## 1.Setup
### build the packages in your ws
Copy all files in sim_env_navigation to your workspace src.

```
cd ~your_ws/src
source /opt/ros/iron/setup.bash
colcon build
```

## 2.Edit file path
There was some code that didn't work unless it was an absolute path, so it needs to be rewritten. Rewrite it to fit in your ws environment.
### nav_rover_control
- URDF   
  - ares8_rover.urdf
    - line:180 <mesh_filename="/home/karisora/nav_ws/install/nav_rover_control/share/nav_rover_control/meshes/base_link.stl" scale="0.001 0.001 0.001"/>
    - line:560 <parameters>/home/karisora/nav_ws/install/nav_rover_control/share/nav_rover_control/config/ares8_rover.yaml</parameters>
    - Rewrite all of them as there are multiple locations.   

    build again.

## 3. Launch nav_simulation
> [!IMPORTANT]
> Plz source bash files before launch .py

### launch navigation package
launch gps follower navigation node.
```
ros2 launch nav_rover_control gps_waypoint_follower.launch.py
```

### launch waypoint follow
GPS data is converted to odometry.
The rover goes to the location set in the yaml file.
> [!IMPORTANT]
> Plz change .yaml url in your env.

```
ros2 run nav_rover_control logged_waypoint_follower /home/karisora/ros2_ws/src/nav_rover_control/config/demo_waypoints.yaml
```
