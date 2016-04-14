#include <can_searcher/can_searcher.h>
#include <iostream>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h> 

using namespace std;

CanSearcher::CanSearcher(ros::NodeHandle* nodehandle): nh_(*nodehandle), Pclutils_(nodehandle),
tf_downsampled_ptr_(new pcl::PointCloud<pcl::PointXYZRGB>)
{
	ROS_INFO("Initializing Publishers");
	pubSPKinect_ = nh_.advertise<sensor_msgs::PointCloud2>("downsampled_kinect",1,true);
	pubTable_ = nh_.advertise<sensor_msgs::PointCloud2>("table",1,true);
	pubCanTop_ = nh_.advertise<sensor_msgs::PointCloud2>("can_top",1,true);

    tf::StampedTransform tf_sensor_frame_to_torso_frame; //use this to transform sensor frame to torso frame
    tf::TransformListener tf_listener; //start a transform listener

    // let's warm up the tf_listener, to make sure it get's all the transforms it needs to avoid crashing:
    bool tferr = true;
    ROS_INFO("waiting for tf between kinect_pc_frame and torso...");
    while (tferr) {
        tferr = false;
        try {
            // The direction of the transform returned will be from the target_frame to the source_frame. 
            // Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
        } catch (tf::TransformException &exception) {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep(); // sleep for half a second
            ros::spinOnce();
        }
    }
    ROS_INFO("tf is good"); //  tf-listener found a complete chain from sensor to world; ready to roll
    kinect_wrt_torso_ = Pclutils_.transformTFToEigen(tf_sensor_frame_to_torso_frame);
}

void CanSearcher::takeAPic()
{
	// check the kinect points
	ros::spinOnce();
	while(!Pclutils_.got_kinect_cloud())
	{
		ROS_WARN("Wait for kinect ...");
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
	ROS_INFO("Connected kinect.");
	Pclutils_.reset_got_kinect_cloud();

	// get the kinect points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	Pclutils_.get_kinect_points(*kinect_ptr);

	// get the downsampled points
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(kinect_ptr);
    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_ptr);
    cout << "num bytes in original cloud data = " << kinect_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_ptr->points.size() << endl; 
    Pclutils_.transform_cloud(kinect_wrt_torso_,downsampled_ptr,tf_downsampled_ptr_);
}

bool searchTable()
{

}

bool searchCan(Eigen::Vector3f &centroid)
{

}

void CanSearcher::publishPoints()
{
	// update frames
	kinect_cloud_.header.frame_id = "torso";
	table_cloud_.header.frame_id = "torso";
	can_top_cloud_.header.frame_id = "torso";
	// publish all
	pubSPKinect_.publish(kinect_cloud_);
	pubTable_.publish(table_cloud_);
	pubCanTop_.publish(can_top_cloud_);

}

void CanSearcher::savePCD()
{
	// check the kinect points
	ros::spinOnce();
	while(!Pclutils_.got_kinect_cloud())
	{
		ROS_WARN("Wait for kinect ...");
		ros::Duration(0.5).sleep();
		ros::spinOnce();
	}
	ROS_INFO("Connected kinect.");
	Pclutils_.reset_got_kinect_cloud();

	// save PCD
	Pclutils_.save_kinect_clr_snapshot();
}
