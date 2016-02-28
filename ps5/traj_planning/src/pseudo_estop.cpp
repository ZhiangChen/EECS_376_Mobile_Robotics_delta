#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "pseudo_estop"); // name of this node will be "minimal_publisher"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Bool>("motors_enabled", 1);
    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    
    std_msgs::Bool estop;

    estop.data = true;
    
    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
	    char d='d';	
   		while(d!='y' && d!='x' && d!='q')
        {
            cout<<"please input y to trigger estop, x to clear estop, or q to quit:"<<endl;
            cin>>d;	
        }   
        if (d=='y')
        {
            estop.data=false;
            my_publisher_object.publish(estop);
        }
        if (d=='x')
        {
            estop.data=true;
            my_publisher_object.publish(estop);
        }
        if (d=='q')
            return 0;

    }
}

