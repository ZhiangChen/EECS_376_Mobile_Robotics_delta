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
    ROS_INFO("waiting for tf between camera and torso...");
    while (tferr) {
        tferr = false;
        try {
            // The direction of the transform returned will be from the target_frame to the source_frame. 
            // Which if applied to data, will transform data in the source_frame into the target_frame. See tf/CoordinateFrameConventions#Transform_Direction
            #ifndef Gazebo_baxter
            tf_listener.lookupTransform("base", "camera_depth_optical_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
            #else 
            tf_listener.lookupTransform("torso", "kinect_pc_frame", ros::Time(0), tf_sensor_frame_to_torso_frame);
            #endif
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
    pcl::toROSMsg(*tf_downsampled_ptr_, kinect_cloud_);
}

bool CanSearcher::searchTable()
{
	got_table_ = false;
	vector<int> indices_table_plane;
    vector<int> indices_table;
    // 1. find the plane of the talbe
    ROS_INFO("Searching the table...");
    indices_table_plane.clear();
	Pclutils_.filter_cloud_z(tf_downsampled_ptr_,TableHeight,TableTol,indices_table_plane);
	// 2. find the table
	int n = indices_table_plane.size();
	if (n<20)
	{
		ROS_WARN("Failed to find the table plane!");
		return false;
	}
	Eigen::Vector3f pnt;
	Eigen::Vector3f clr;
	Eigen::Vector3f t_clr;
	double clr_error;
	t_clr<<TableRed,TableGreen,TableBlue;
	indices_table.clear();
	for (int i=0; i<n; i++)
	{
		pnt = tf_downsampled_ptr_->points[indices_table_plane[i]].getVector3fMap();
		if (pnt[1]>TableLeft && pnt[1]<TableRight) // filter along y axil
			if(pnt[0]>TableBottom && pnt[0]<TableTop) // filter along x axil
			{
				clr[0]=tf_downsampled_ptr_->points[indices_table_plane[i]].r;
				clr[1]=tf_downsampled_ptr_->points[indices_table_plane[i]].g;
				clr[2]=tf_downsampled_ptr_->points[indices_table_plane[i]].b;
				clr_error=(clr-t_clr).norm();
				if (clr_error<ClrTol)
				{
					indices_table.push_back(indices_table_plane[i]);
				}
			}
	}
	n = indices_table.size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	Pclutils_.copy_cloud_xyzrgb_indices(tf_downsampled_ptr_,indices_table,table_ptr);
	pcl::toROSMsg(*table_ptr, table_cloud_);
	ROS_INFO("Got the table with %d points.",n);
	if (n<20)
	{
		ROS_WARN("Failed to find the table!");
		return false;
	}
	// find the centroid
	Eigen::Vector3f cloud_pt;   
    n = indices_table.size();    
    table_centroid_<<0,0,0;
    for (int i = 0; i < n; i++) {
        cloud_pt = tf_downsampled_ptr_->points[indices_table[i]].getVector3fMap();
        table_centroid_ += cloud_pt; //add all the column vectors together
    }
    table_centroid_/= n; //divide by the number of points to get the centroid
    ROS_INFO_STREAM("The centroid of the table is "<<table_centroid_.transpose());
    got_table_ = true;
    // 3. grid for table 
    float l_m = 10;
    float r_m = -10;
    float t_m = -10;
    float b_m = 10;
    for (int i = 0; i < n; i++) {
        cloud_pt = tf_downsampled_ptr_->points[indices_table[i]].getVector3fMap();
        if(cloud_pt[0]<b_m)
        	b_m = cloud_pt[0];
        if(cloud_pt[0]>t_m)
        	t_m = cloud_pt[0];
        if(cloud_pt[1]<l_m)
        	l_m = cloud_pt[1];
        if(cloud_pt[1]>r_m)
        	r_m = cloud_pt[1];
    }
    table_corners_ << l_m,r_m,t_m,b_m;
    ROS_INFO_STREAM("The boundary of the table(left,right,top,bottom):"<<table_corners_.transpose());
    return true;
}


// If the can is found out, it will return true and the centroid the can
// Otherwise, it will return false
bool CanSearcher::searchCan(Eigen::Vector3f &centroid)
{
	if (!got_table_)
	{
		ROS_WARN("Can't search the can without the table!");
		return false;
	}
	ROS_INFO("Searching the coke can...");
	vector<int> indices_can;
	int n = tf_downsampled_ptr_->points.size();
	Eigen::Vector3f pnt;
	Eigen::Vector3f dist;
	for (int i=0; i<n; i++)
	{
		pnt = tf_downsampled_ptr_->points[i].getVector3fMap();
		dist = pnt - table_centroid_;
		if (dist[0]>-TableWidth/2.0 && dist[0]<TableWidth/2.0)
			if (dist[1]>-TableLength/2.0 && dist[1]<TableLength/2.0)
				if (dist[2]>CanHeight)
				{
					indices_can.push_back(i);
				}
	}
	n = indices_can.size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	Pclutils_.copy_cloud_xyzrgb_indices(tf_downsampled_ptr_,indices_can,can_ptr);
	pcl::toROSMsg(*can_ptr, can_top_cloud_);
	ROS_INFO("Got the can with %d points",n);
	if (n<8)
	{
		ROS_WARN("Failed to find the can!");
		return false;
	}
	// find the centroid of the top of the can	
	Eigen::Vector3f cloud_pt;   
    Eigen::Vector3f t_centroid;    
    t_centroid<<0,0,0;
    for (int i = 0; i < n; i++) {
        cloud_pt = tf_downsampled_ptr_->points[indices_can[i]].getVector3fMap();
        t_centroid += cloud_pt; //add all the column vectors together
    }
    t_centroid/= n; //divide by the number of points to get the centroid
    centroid = t_centroid;
    centroid[2] -= CanHeight/2; 
    ROS_INFO_STREAM("The centroid of the can is "<<centroid.transpose());
    return true;
}

bool CanSearcher::searchCan2(Eigen::Vector3f &centroid)
{
	if (!got_table_)
	{
		ROS_WARN("Can't search the can without the table!");
		return false;
	}
	ROS_INFO("Searching the coke can...");
    float l_m = table_corners_[0];
    float r_m = table_corners_[1];
    float t_m = table_corners_[2];
    float b_m = table_corners_[3];	

    vector<int> indices_can;
	int n = tf_downsampled_ptr_->points.size();
	Eigen::Vector3f pnt;
	Eigen::Vector3f dist;
	for (int i=0; i<n; i++)
	{
		pnt = tf_downsampled_ptr_->points[i].getVector3fMap();
		dist = pnt - table_centroid_;
		if (pnt[0]>b_m && pnt[0]<t_m)
			if (pnt[1]>l_m && pnt[1]<r_m)
				if (dist[2]>CanHeight && dist[2]<CanHeight+0.08)
				{
					indices_can.push_back(i);
				}
	}
	n = indices_can.size();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr can_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	Pclutils_.copy_cloud_xyzrgb_indices(tf_downsampled_ptr_,indices_can,can_ptr);
	pcl::toROSMsg(*can_ptr, can_top_cloud_);
	ROS_INFO("Got the can with %d points",n);
	if (n<8)
	{
		ROS_WARN("Failed to find the can!");
		return false;
	}
	// find the centroid of the top of the can	
	Eigen::Vector3f cloud_pt;   
    Eigen::Vector3f t_centroid;    
    t_centroid<<0,0,0;
    for (int i = 0; i < n; i++) {
        cloud_pt = tf_downsampled_ptr_->points[indices_can[i]].getVector3fMap();
        t_centroid += cloud_pt; //add all the column vectors together
    }
    t_centroid/= n; //divide by the number of points to get the centroid
    centroid = t_centroid;
    centroid[2] -= CanHeight/2; 
    centroid[2] += Z_OFFSET;
    centroid[0] += Z_OFFSET;
    ROS_INFO_STREAM("The centroid of the can is "<<centroid.transpose());
    return true;
}


void CanSearcher::publishPoints()
{
	// update frames
	#ifndef Gazebo_baxter
	kinect_cloud_.header.frame_id = "base";
	table_cloud_.header.frame_id = "base";
	can_top_cloud_.header.frame_id = "base";
	#else
	kinect_cloud_.header.frame_id = "torso";
	table_cloud_.header.frame_id = "torso";
	can_top_cloud_.header.frame_id = "torso";
	#endif
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
