/*
EECS 476 final project
This header is used with move_jinx.cpp
Written by Ananya, 4/14/16, 14:25
*/

#ifndef MOVEJINX_H_
#define MOVEJINX_H_
#include <ros/ros.h> //generic C++ stuff
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <pub_des_state/path.h>
#include <std_msgs/Float64.h>
#include <pub_des_state/ps5_pub_des_state.h>
#define left_table_dist 1.0
#define right_table_dist 1.0
// define a class, including a constructor, member variables and member functions
class MoveJinx
{
public:
    MoveJinx(ros::NodeHandle* nodehandle);//"main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    // may choose to define public methods or public variables, if desired
    bool move2LT(); // move to the left table;
    bool moveBack(); // move to the initial position;
    bool move2RT();  // move to the right table;


private:
DesStatePublisher desStatePublisher;
ros::NodeHandle nh_;

} ;

#endif

