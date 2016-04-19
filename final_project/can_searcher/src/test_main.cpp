// p9_zxc251
// Zhiang Chen, 2016

#include<ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> //to convert between PCL and ROS
#include <pcl/ros/conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
//#include <pcl/PCLPointCloud2.h> //PCL is migrating to PointCloud2 

#include <pcl/common/common_headers.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/PCLHeader.h>

//will use filter objects "passthrough" and "voxel_grid" in this example
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

#include <pcl_utils/pcl_utils.h>  //a local library with some utility fncs
#include <can_searcher/can_searcher.h>
using namespace std;

int main(int argc, char** argv) {
    ros::init(argc, argv, "coke_can_finder"); //node name
    ros::NodeHandle nh;
    CanSearcher Can_searcher(&nh);

    Can_searcher.takeAPic();
    
    Eigen::Affine3f A_plane_wrt_camera;
    A_plane_wrt_camera.matrix()(0,0)=0.999679;
    A_plane_wrt_camera.matrix()(0,1)=0;
    A_plane_wrt_camera.matrix()(0,2)=-0.025333;
    A_plane_wrt_camera.matrix()(0,3)=0.0247843;

    A_plane_wrt_camera.matrix()(1,0)=-0.00400895;
    A_plane_wrt_camera.matrix()(1,1)=-0.986884;
    A_plane_wrt_camera.matrix()(1,2)=-0.161378;
    A_plane_wrt_camera.matrix()(1,3)=0.157883;

    A_plane_wrt_camera.matrix()(2,0)=-0.025007;
    A_plane_wrt_camera.matrix()(2,1)=0.16143;
    A_plane_wrt_camera.matrix()(2,2)=-0.986567;
    A_plane_wrt_camera.matrix()(2,3)=0.9651992;

    A_plane_wrt_camera.matrix()(3,0)=0;
    A_plane_wrt_camera.matrix()(3,1)=0;
    A_plane_wrt_camera.matrix()(3,2)=0;
    A_plane_wrt_camera.matrix()(3,3)=1;
    
    //Can_searcher.setAffine(A_plane_wrt_camera);

    bool got_table;
    bool got_can;
    Eigen::Vector3f can_centroid;
    got_table = Can_searcher.searchTable();
    if (got_table)
    {
        got_can = Can_searcher.searchCan2(can_centroid);
    }
    if (got_can)
    {
        ROS_INFO("Got the coke can");
        ROS_INFO_STREAM("Its centroid is"<<can_centroid.transpose());        
    }

    Can_searcher.savePCD();
    // Eigen::Affine3f af;
    // Can_searcher.setAffine(af);

    while (ros::ok()) {
        // publish all pc
        Can_searcher.publishPoints();
        ros::spinOnce(); 
        ros::Duration(0.1).sleep();
    }

    return 0;
}

