#include <ros/ros.h> //generic C++ stuff
#include <iostream>
#include <move_jinx/move_jinx.h>
#include <pub_des_state/ps5_pub_des_state.h>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_jinx");
    ros::NodeHandle nh;
    
    MoveJinx Movejinx(&nh);

}
