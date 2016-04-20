// Program to move jinx forward to the right table, reverse back and go to the left table 
// The left table distance and heading, and the right table distance and heading can be adjusted in the header move_jinx.h

// Written by Ananya, 4/14/16, 14:44
#include <pub_des_state/ps5_pub_des_state.h>
#include <iostream>
#include <move_jinx/move_jinx.h>
using namespace std;

MoveJinx::MoveJinx(ros::NodeHandle* nodehandle): nh_(*nodehandle), desStatePublisher(*nodehandle)
{ 
	//setting inirial pose
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
}

   // ros::Rate looprate(1 / dt);
int i = 0;
// moving jinx to the left table
bool MoveJinx::move2LT()
{    
	/*
	cout<<"Enter the left table distance :"<<endl;
	cin>> left_table_dist;
	cout<<"Enter the left table heading :"<<endl;
	cin>> left_table_heading;
	*/
    desStatePublisher.append_path_queue(left_table_dist,0.0,left_table_heading);
    char d='x';	
    while(d!='y')
    {
        cout<<"please input y to move to the left table or x to quit:"<<endl;
        cin>>d;	
        if (d=='x')
        	{   ROS_INFO("Aborted. ") ;
            return 0;
        }
        else 
        	{ 
        		ROS_INFO("Moving to the left table...") ;
            }
    }   


    while (ros::ok())
    {
        desStatePublisher.pub_next_state();
    	if(desStatePublisher.get_estop_trigger())
    	{
    		ROS_INFO("estop is triggered");
    	}
	    i+=1;
	     if (i>=1000){
		break;
		}
            //ros::spinOnce();
ros::Duration(0.02).sleep();            
//looprate.sleep(); //sleep for defined sample period, then do loop again
    }
    return true;
}
// moving jinx to the initial position
bool MoveJinx::moveBack()
{
   //desStatePublisher.append_path_queue(left_table_dist,0.0,0.0);
   desStatePublisher.append_path_queue(0.0,0.0,0.0);
    char d='x';	
    while(d!='y')
    {
        cout<<"please input y to move back to initial position or x to quit:"<<endl;
        cin>>d;	
        if (d=='x'){
        	ROS_INFO("Aborted. ") ;
            return 0;
        }
        else 
        	{ ROS_INFO("Moving back...") ;}
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
// moving jinx to the right table
bool MoveJinx::move2RT()
{
		/*
	cout<<"Enter the right table distance :"<<endl;
	cin>> right_table_dist;
	cout<<"Enter the right table heading :"<<endl;
	cin>> right_table_heading;
	*/
	desStatePublisher.append_path_queue(0.0,right_table_dist,right_table_heading);
    char d='x';	
    while(d!='y')
    {
        cout<<"please input y to go to the right table or x to quit:"<<endl;
        cin>>d;	
        if (d=='x'){
        	ROS_INFO("Aborted. ") ;
            return 0;
        }else
        { ROS_INFO("Moving to the right table...") ;}
    }   
    while (ros::ok())
    {
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









