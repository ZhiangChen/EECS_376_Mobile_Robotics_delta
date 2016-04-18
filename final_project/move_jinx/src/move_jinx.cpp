// Program to move jinx forward to the right table, reverse back and go to the left table goes here

// Written by Ananya, 4/14/16, 14:44
#include <pub_des_state/ps5_pub_des_state.h>
#include <iostream>
#include <ros/ros.h> //generic C++ stuff
using namespace std;

// This header incorporates all the necessary #include files and defines the class MoveJinx

#include <move_jinx/move_jinx.h>

MoveJinx::MoveJinx(ros::NodeHandle* nodehandle): nh_(*nodehandle)
{ 
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
    MoveJinx::move2LT();
    MoveJinx::moveBack();
    MoveJinx::move2RT();
}

bool MoveJinx::move2LT()
{    
    desStatePublisher.append_path_queue(0.0, left_table_dist,0.0);
    return true;
}
bool MoveJinx::moveBack()
{
   desStatePublisher.append_path_queue(0.0,0.0,0.0);
   return true;
}
bool MoveJinx::move2RT()
{  desStatePublisher.append_path_queue(right_table_dist,0.0,0.0);
   return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_jinx");
    ros::NodeHandle nh;
    //instantiate a desired-state publisher object
    MoveJinx::MoveJinx (&nh);
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









