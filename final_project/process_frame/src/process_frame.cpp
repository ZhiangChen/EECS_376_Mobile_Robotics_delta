#include <can_searcher/can_searcher.h>
#include <move_jinx/move_jinx.h>
#include <move_arm/move_arm.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "process_frame");
    ros::NodeHandle nh;
    CanSearcher Cansearcher(&nh);
    MoveJinx Movejinx(&nh);
    MoveArm Movearm(&nh);


    while (ros::ok()) {

    }
}


