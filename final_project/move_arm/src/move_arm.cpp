#include <move_arm/move_arm.h>


MoveArm::MoveArm(ros::NodeHandle* nh):nh_(nh) 
{

}

bool MoveArm::setPrepose(){

return true;
}

bool MoveArm::graspCan(Eigen::Vector3f centroid){

return true;
}

bool MoveArm::moveCan(){

return true;
}

bool MoveArm::dropCan(){

return true;
}
