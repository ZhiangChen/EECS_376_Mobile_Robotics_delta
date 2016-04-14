#include <can_searcher/can_searcher.h>
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "process_frame");
    ros::NodeHandle nh;
    CanSearcher Cansearcher(&nh);

    while (ros::ok()) {

        }
}


