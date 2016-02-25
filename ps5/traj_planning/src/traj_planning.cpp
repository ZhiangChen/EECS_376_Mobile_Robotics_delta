#include <pub_des_state/ps5_pub_des_state.h>
#include <traj_planning/subgoals_msg.h>


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
    desStatePublisher.append_path_queue(5.0,5.0,0.0);
    desStatePublisher.append_path_queue(0.0,5.0,0.0);
    desStatePublisher.append_path_queue(0.0,0.0,0.0);
    
    std::vector<nav_msgs::Odometry> subgoals;
    // main loop; publish a desired state every iteration
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


