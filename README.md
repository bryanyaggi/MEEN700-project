# MEEN 700 Project: Radio-Based Localization  

## Instructions
1. Download ROS bag files into `~/bags`. Alternatively use another location and update the volume in `local.yml`.  

2. Build Docker image.  
`cd docker`  
`docker compose -f local.yml build`  

3. Start Docker container: `./run.sh`.  

4. Build workspace: `catkin build`.  

5. Source workspace: `source devel/setup.bash`.  

5. Launch visualization: `roslaunch radio_localization small_scenario.launch`. Use `large_scenario.launch` for large
   scenario.  

6. Get another shell session in container: `./shell`. Repeat 5.  

7. Play bag file in another shell: `rosbag play ../bags/<ROS bag file>`.  
