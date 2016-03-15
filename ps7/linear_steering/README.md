# lin_steering
Steering algorithm using odometry.  For mobot, "odom" is perfect.  Neede to relax this
assumption.

If start with good initial conditions, linear steering algorithm will do a good job.
Can compare feedback controller to open-loop controller.

## Example usage
Start up gazebo, load the mobot model, desired-state publisher, desired-state client,
and linear-steering algorithm.
`roslaunch ps7 ps7.launch`
`rosrun ps6 ps6`


    
