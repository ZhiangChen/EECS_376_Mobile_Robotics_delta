// Program to move jinx forward to the right table, reverse back and go to the left table goes here

// Written by Ananya, 4/14/16, 14:44
#include <pub_des_state/ps5_pub_des_state.h>
#include <iostream>
#include <move_jinx/move_jinx.h>
using namespace std;

// This header incorporates all the necessary #include files and defines the class MoveJinx


MoveJinx::MoveJinx(ros::NodeHandle* nodehandle): nh_(*nodehandle), desStatePublisher(*nodehandle)
{ 
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
}

   // ros::Rate looprate(1 / dt);

bool MoveJinx::move2LT()
{    
    desStatePublisher.append_path_queue(left_table_dist,0.0,0.0);
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
ros::Duration(0.02).sleep();            
//looprate.sleep(); //sleep for defined sample period, then do loop again
        }
    return true;
}
bool MoveJinx::moveBack()
{
   desStatePublisher.append_path_queue(left_table_dist,0.0,0.0);
   desStatePublisher.append_path_queue(0.0,0.0,0.0);
   desStatePublisher.append_path_queue(0.0,right_table_dist,0.0);
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
ros::Duration(0.02).sleep();            
//looprate.sleep(); //sleep for defined sample period, then do loop again
        }
   return true;
}









