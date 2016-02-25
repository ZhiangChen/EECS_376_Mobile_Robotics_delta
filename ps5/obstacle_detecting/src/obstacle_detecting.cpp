#include <pub_des_state/ps5_pub_des_state.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
bool g_lidar_alarm;
void obstacle_detecting(const std_msgs::bool& alarm) 
{
g_lidar_alarm = alarm; //Sets g_lidar_alarm to the data from the subscriber
} 
int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_planning");
    ros::NodeHandle n;
ros::Subscriber my_subscriber_object= n.subscribe("lidar_alarm",1,obstacle_detecting);
ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("estop_service");
ros::ServiceClient client2 = n.serviceClient<std_srvs::Trigger>("clear_estop_service");
std_srvs::Trigger srv; //Sets up estop.
while(ros::ok()
{
	ros::spinOnce();
	double wait = 4.0; //Robot will be waiting 4 seconds to see if object clears
	ros::Rate loop_timer(wait); 
	if (g_lidar_alarm==true){ //If the alarm goes off, the e_stop needs to be triggered
		if (client.call(srv)){ //triggers e_stop
			loop_timer.sleep(); //If the alarm is set off, it will wait 4 seconds to see if the obstacle moves.
			if (g_lidar_alarm==false){ //If the alarm turns off, the e_stop is not triggered
				if (client2.call(srv)){ //Resets the e_stop
				}
			}
		}
	}
}
ros::spin();
return 0;//Should not get here  
}
