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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //pointer for color version of pointcloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); //ptr to hold filtered Kinect image
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_kinect_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_downsampled_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
    vector<int> indices;
	
    CanSearcher CS(&nh);
    //load a PCD file using pcl::io function; alternatively, could subscribe to Kinect messages    
    string fname;
    fname = "coke_can.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fname, *kinect_ptr) == -1) //* load the file
    {
        ROS_ERROR("Couldn't read file \n");
        return (-1);
    }
    //PCD file does not seem to record the reference frame;  set frame_id manually
    kinect_ptr->header.frame_id = "camera_frame";

    //will publish  pointClouds as ROS-compatible messages; create publishers; note topics for rviz viewing
    ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/pcd", 1);
    ros::Publisher pubDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("/downsampled_pcd", 1);
    ros::Publisher pubTFCloud = nh.advertise<sensor_msgs::PointCloud2> ("/tf_pcd", 1);
    ros::Publisher pubTFDnSamp = nh.advertise<sensor_msgs::PointCloud2> ("/tf_downsampled_pcd", 1);
    ros::Publisher pubStlPlane = nh.advertise<sensor_msgs::PointCloud2> ("/tf_stool", 1);


    PclUtils pclUtils(&nh); 

    Eigen::Affine3f A_plane_wrt_camera;
    A_plane_wrt_camera.matrix()(0,0)=0.997338;
    A_plane_wrt_camera.matrix()(0,1)=0;
    A_plane_wrt_camera.matrix()(0,2)=-0.0729215;
    A_plane_wrt_camera.matrix()(0,3)=0.0335546;

    A_plane_wrt_camera.matrix()(1,0)=-0.0622635;
    A_plane_wrt_camera.matrix()(1,1)=-0.52053;
    A_plane_wrt_camera.matrix()(1,2)=-0.85157;
    A_plane_wrt_camera.matrix()(1,3)=0.391847;

    A_plane_wrt_camera.matrix()(2,0)=-0.0379578;
    A_plane_wrt_camera.matrix()(2,1)=0.853843;
    A_plane_wrt_camera.matrix()(2,2)=-0.519144;
    A_plane_wrt_camera.matrix()(2,3)=0.238883;

    A_plane_wrt_camera.matrix()(3,0)=0;
    A_plane_wrt_camera.matrix()(3,1)=0;
    A_plane_wrt_camera.matrix()(3,2)=0;
    A_plane_wrt_camera.matrix()(3,3)=1;

    double table_r = 0.16;

    sensor_msgs::PointCloud2 kinect_cloud, downsampled_cloud, transformed_downsampled_cloud, transformed_kinect_cloud; //here are ROS-compatible messages
    pcl::transformPointCloud(*kinect_ptr, *transformed_kinect_ptr, A_plane_wrt_camera.inverse());
    pcl::toROSMsg(*kinect_ptr, kinect_cloud);
    pcl::toROSMsg(*transformed_kinect_ptr, transformed_kinect_cloud); //convert from PCL cloud to ROS message this way

    //use voxel filtering to downsample the original cloud:
    cout << "starting voxel filtering" << endl;
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(kinect_ptr);
    vox.setLeafSize(0.02f, 0.02f, 0.02f);
    vox.filter(*downsampled_ptr);
    cout << "done voxel filtering" << endl;
    cout << "num bytes in original cloud data = " << kinect_ptr->points.size() << endl;
    cout << "num bytes in filtered cloud data = " << downsampled_ptr->points.size() << endl; // ->data.size()<<endl;    

    pcl::transformPointCloud(*downsampled_ptr, *transformed_downsampled_ptr , A_plane_wrt_camera.inverse());
    pcl::toROSMsg(*downsampled_ptr, downsampled_cloud);
    pcl::toROSMsg(*transformed_downsampled_ptr, transformed_downsampled_cloud); //convert to ros message for publication and display



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stool_plane_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stool_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
    sensor_msgs::PointCloud2 stool_cloud;
    vector<int> indices_stool_plane;
    vector<int> indices_stool;
    vector<int> indices_coke_can;

    /* use downsampled cloud to find the stool and the coke can. */
    /* filter cloud along z axil */
    // 1. find the plane of the stool
    pclUtils.filter_cloud_z(transformed_downsampled_ptr,0.0,0.01,indices_stool_plane);
    // 2. find the grid of the stool in grids
    // find the point on the bottom (along y axil) of the stool
        // this is necessary when the bottom point isn't the point on the stool
        /*pcl::PassThrough<pcl::PointXYZRGB> pass; 
        pass.setInputCloud(transformed_cloud_ptr); 
        pass.setFilterFieldName("z"); 
        pass.setFilterLimits(-0.02, 0.02); 
        pass.filter(indices); */
    int n = indices_stool_plane.size();
    Eigen::Vector3f bottom_pnt;
    bottom_pnt<<0,1.0,0;
    for (int i=0; i<n; i++)
    {
        if(transformed_downsampled_ptr->points[indices_stool_plane[i]].y<bottom_pnt[1])
        {
            bottom_pnt = transformed_downsampled_ptr->points[indices_stool_plane[i]].getVector3fMap();  
        }
    }
    //cout<<bottom_pnt;
    // find the centroid of the stool
    Eigen::Vector3f center_pnt;
    Eigen::Vector3f temp_pnt;
    center_pnt[0]=bottom_pnt[0];
    center_pnt[1]=bottom_pnt[1]+table_r;
    center_pnt[2]=bottom_pnt[2];
    // get the points on the stool
    double dist;
    indices_stool.clear();
    for (int i=0; i<n; i++)
    {
        temp_pnt=center_pnt - transformed_downsampled_ptr->points[indices_stool_plane[i]].getVector3fMap();
        dist = temp_pnt.norm();
        if (dist < table_r+0.03)
        {
            indices_stool.push_back(indices_stool_plane[i]);
        }
    }
    pclUtils.copy_cloud_xyzrgb_indices(transformed_downsampled_ptr,indices_stool,stool_ptr);
    pcl::toROSMsg(*stool_ptr, stool_cloud);
    // 3. get the normal vector of the stool in camera frame
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr stool_clr_wrt_cmr_ptr(new pcl::PointCloud<pcl::PointXYZRGB>); 
    pclUtils.copy_cloud_xyzrgb_indices(downsampled_ptr,indices_stool,stool_clr_wrt_cmr_ptr);
    float curvature;
    Eigen::Vector4f plane_parameters;
    pcl::computePointNormal(*stool_clr_wrt_cmr_ptr, plane_parameters, curvature); 
    cout << "The normal vector of the stool wrt to the camera frame: (" 
    << plane_parameters.transpose()[0]<<","
    << plane_parameters.transpose()[1]<<","
    << plane_parameters.transpose()[2]<<")" << endl;
    A_plane_wrt_camera = pclUtils.make_affine_from_plane_params(plane_parameters);
    // 4. compute the height of the camera from the stool plane wrt the stool frame
    Eigen::Vector3f camera_origin;
    camera_origin<<0.0,0.0,0.0;
    camera_origin= A_plane_wrt_camera.inverse()*camera_origin;
    cout<< "The height of the camera from the stool plane wrt the stool frame: "<<camera_origin[2]<<endl;
    // 5. find the centroid of the top of the coke can
    // find the points above the stool
    double pnt_z;
    n = transformed_downsampled_ptr->size();
    indices_coke_can.clear();
    for (int i=0;i<n;i++)
    {
        pnt_z=transformed_downsampled_ptr->points[i].z;
        if(pnt_z>center_pnt[2]+0.11 && pnt_z<center_pnt[2]+0.14)
        {
            temp_pnt=center_pnt - transformed_downsampled_ptr->points[i].getVector3fMap();
            temp_pnt[2]=0;
            dist=temp_pnt.norm();
            if(dist<table_r+0.03)
            {
                indices_coke_can.push_back(i);
            }
        }
    }
    // find the centroid 
    Eigen::Vector3f centroid;
    Eigen::Vector3f cloud_pt;   
    n = indices_coke_can.size();    
    centroid<<0,0,0;
    for (int i = 0; i < n; i++) {
        cloud_pt = transformed_downsampled_ptr->points[indices_coke_can[i]].getVector3fMap();
        centroid += cloud_pt; //add all the column vectors together
    }
    centroid/= n; //divide by the number of points to get the centroid
    centroid = A_plane_wrt_camera*centroid;
    cout<<"The centroid of the coke can's top: "<<centroid.transpose()<<endl;

    // update all the frames
    kinect_cloud.header.frame_id = "camera_frame";
    downsampled_cloud.header.frame_id = "camera_frame";
    transformed_kinect_cloud.header.frame_id = "stool_frame";
    transformed_downsampled_cloud.header.frame_id = "stool_frame";
    stool_cloud.header.frame_id = "stool_frame";

    while (ros::ok()) {
        // publish all pc
        pubStlPlane.publish(stool_cloud);
        pubCloud.publish(kinect_cloud);
        pubDnSamp.publish(downsampled_cloud);
        pubTFCloud.publish(transformed_kinect_cloud); // will not need to keep republishing if display setting is persistent
        pubTFDnSamp.publish(transformed_downsampled_cloud); //can directly publish a pcl::PointCloud2!!
        ros::spinOnce(); //pclUtils needs some spin cycles to invoke callbacks for new selected points
        ros::Duration(0.1).sleep();
    }

    return 0;
}

