#include <ros/ros.h> //generic C++ stuff
#include <iostream>
#include <move_jinx/move_jinx.h>
#include <pub_des_state/ps5_pub_des_state.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_jinx");
    ros::NodeHandle nh;
    
    MoveJinx Movejinx(&nh);
    //dt is set in header file pub_des_state.h    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate

    // main loop; publish a desired state every iteration
    char d='x';	
    while(d!='y')
    {
        cout<<"please input y to execute the path, or x to quit:"<<endl;
        cin>>d;	
        if (d=='x')
            return 0;
    }

    while (ros::ok()) {
       desStatePublisher.pub_next_state();
    	if(desStatePublisher.get_estop_trigger())
    	{
    		ROS_INFO("estop is triggered");
    	}
            ros::spinOnce();
            looprate.sleep(); //sleep for defined sample period, then do loop again
        }
}
