#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>

bool g_lidar_alarm=false;
void obstacle_detecting(const std_msgs::Bool& alarm) 
{
	g_lidar_alarm = alarm.data; //Sets g_lidar_alarm to the data from the subscriber
} 
int main(int argc, char **argv) {
    ros::init(argc, argv, "obstacle_detecting");
    ros::NodeHandle n;
	ros::Subscriber my_subscriber_object= n.subscribe("lidar_alarm",1,obstacle_detecting);
	ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("estop_service");
	ros::ServiceClient client2 = n.serviceClient<std_srvs::Trigger>("clear_estop_service");
	std_srvs::Trigger srv; //Sets up estop.

	while (!client.exists()) {
      ROS_INFO("waiting for service...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client to service");

    while (!client2.exists()) {
      ROS_INFO("waiting for service2...");
      ros::Duration(1.0).sleep();
    }
    ROS_INFO("connected client2 to service");

	while(ros::ok())
	{
		ros::spinOnce();
	
		if (g_lidar_alarm==true){ //If the alarm goes off, the e_stop needs to be triggered
			if (client.call(srv)){ //triggers e_stop
				ROS_INFO("waiting for the obstacle");
				ros::Duration(10.0).sleep();
				ros::spinOnce();
				if (g_lidar_alarm==false){ //If the alarm turns off, the e_stop is not triggered
					if (client2.call(srv)){ //Resets the e_stop
					}
				}
			}
		}
	}
	return 0;//Should not get here  
}
