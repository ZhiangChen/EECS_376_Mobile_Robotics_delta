Chen: interface
Xiangyu: braking
Ananya: hardware estop monitoring and recoverying (new node required)
subscriber for hardware estop topic
client for estopServiceCallback
client for clearEstopServiceCallback

Evan: lidar alarm monitoring and recoverying (new node required)
subscriber for lidar alarm
client for estopServiceCallback
client for clearEstopServiceCallback

1) check the interface
roslaunch gazebo_ros empty_world.launch
roslaunch mobot_urdf mobot_w_lidar.launch
rosrun pub_des_state ps5_open_loop_controller
rosrun traj_planning traj_planning

2) check the braking function
estop_service: std_srvs/Trigger:
---
bool success
string message

rosservice call estop_service 

3) check lidar alarm
"lidar_alarm"
"estop_service"

package: "obstacle_detecting"
node: "obstacle_detecting"
rosrun traj_planning ps5_lidar_alarm
(rostopic pub lidar_alarm std_msgs/Bool true)
run node for receiving alarm, preserving current pose and subgoals, triggering e-stop, waiting for 5 seconds, and making a turn to continue subgoals or just continuing subgoals directly.

topic: "obstacle"
bool obs
subgoals

4) check hardware e-stop
package: "hardware_trigger"
node: "hardware_trigger"
subscriber: topic: "motors_enabled": std_msgs/Bool
subscriber: topic: "   ": pose and subgoals
service client: topic "estop_service"

run node for detecting hardware e-stop. if hardware e-stop is triggered, preserve current pose and subgoals. then trigger e-stop. wait for hardware e-stop clearing. When clearing, ask for continuing subgoals or resetting. 





