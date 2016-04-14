#ifndef MOVE_ARM_H_
#define MOVE_ARM_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
class MoveArm
{
public: 
	MoveArm(ros::NodeHandle* nh);	
	bool setPrepose();
    	bool graspCan(Eigen::Vector3f centroid);
   	bool moveCan();
    	bool dropCan();
private:
	ros::NodeHandle* nh_;
};

#endif
