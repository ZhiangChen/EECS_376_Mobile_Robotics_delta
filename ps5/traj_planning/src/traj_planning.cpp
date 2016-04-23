#include <pub_des_state/ps5_pub_des_state.h>
#include <iostream>
using namespace std;


int main(int argc, char **argv) {
    ros::init(argc, argv, "traj_planning");
    ros::NodeHandle nh;
    //instantiate a desired-state publisher object
    DesStatePublisher desStatePublisher(nh);
    //dt is set in header file pub_des_state.h    
    ros::Rate looprate(1 / dt); //timer for fixed publication rate
    desStatePublisher.set_init_pose(0,0,0); //x=0, y=0, psi=0
    //put some points in the path queue--hard coded here


    desStatePublisher.append_path_queue(5.0,0.0,0.0);

    
    // main loop; publish a desired state every iteration
    char d='x';	
    while(d!='y')
    {
        cout<<"please input y to execute the path, or x to quit:"<<endl;
        cin>>d;	
        if (d=='x')
            return 0;
    } 

    int motion_model;  

    

    while (ros::ok()) {
        desStatePublisher.pub_next_state();
    	if(desStatePublisher.get_estop_trigger())
    	{
    		ROS_INFO("estop is triggered");
    	}
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
        motion_model = desStatePublisher.get_motion_mode();
        if (motion_model == DONE_W_SUBGOAL)
            break;

    }
    ROS_ERROR("GOT FIRST");
    desStatePublisher.append_path_queue(5.0,5.0,0.0);

    while (ros::ok()) {
        desStatePublisher.pub_next_state();
        if(desStatePublisher.get_estop_trigger())
        {
            ROS_INFO("estop is triggered");
        }
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
        motion_model = desStatePublisher.get_motion_mode();
        if (motion_model == DONE_W_SUBGOAL)
            break;

    }
    desStatePublisher.append_path_queue(0.0,5.0,0.0);

    while (ros::ok()) {
        desStatePublisher.pub_next_state();
        if(desStatePublisher.get_estop_trigger())
        {
            ROS_INFO("estop is triggered");
        }
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
        motion_model = desStatePublisher.get_motion_mode();
        if (motion_model == DONE_W_SUBGOAL)
            break;

    }
    ROS_ERROR("GOT FIRST");
    desStatePublisher.append_path_queue(0.0,0.0,0.0);   

    while (ros::ok()) {
        desStatePublisher.pub_next_state();
        if(desStatePublisher.get_estop_trigger())
        {
            ROS_INFO("estop is triggered");
        }
        ros::spinOnce();
        looprate.sleep(); //sleep for defined sample period, then do loop again
        motion_model = desStatePublisher.get_motion_mode();
        if (motion_model == DONE_W_SUBGOAL)
            break;

    }
    ROS_ERROR("GOT FIRST");

}


