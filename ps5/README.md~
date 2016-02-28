# PS5 Trajectory Planning

<p>Chen: interface</p>
<p>Xiangyu: braking</p>
<p>Ananya: hardware estop monitoring and recoverying </p>
<p>Evan: lidar alarm monitoring and recoverying</p>

# Usage
<p>roslaunch traj_planning ps5.launch</p>
<p>rosrun traj_planning pseudo_estop</p>
<p>rosrun traj_planning traj_planning</p>

##1) check the interface
<p>roslaunch gazebo_ros empty_world.launch</p>
<p>roslaunch mobot_urdf mobot_w_lidar.launch</p>
<p>rosrun pub_des_state ps5_open_loop_controller</p>
<p>rosrun traj_planning ps5_lidar_alarm</p>
<p>rosrun obstacle_detecting obstacle_detecting</p>
<p>rosrun traj_planning traj_planning</p>


##2) check the braking function
<p>estop_service: std_srvs/Trigger:</p>
<p>---</p>
<p>bool success</p>
<p>string message</p>

<p>rosservice call estop_service</p>

##3) check lidar alarm
<p>"lidar_alarm"</p>
<p>"estop_service"</p>

<p>package: "obstacle_detecting"</p>
<p>node: "obstacle_detecting"</p>
<p>rosrun traj_planning ps5_lidar_alarm</p>
<p>(rostopic pub lidar_alarm std_msgs/Bool true)</p>
<p>run node for receiving alarm, triggering e-stop, waiting for 5 seconds, continuing subgoals directly when the obstacles are removed.</p>


##4) check hardware e-stop
<p>subscriber: topic: "motors_enabled": std_msgs/Bool</p>

<p>run node for detecting hardware e-stop. if hardware e-stop is triggered, then trigger e-stop. wait for hardware e-stop clearing. When clearing, ask for continuing subgoals or resetting. </p>





