//
// Created by aaron on 28/09/18.
//

#include <iostream>
#include <memory>
#include <ros/ros.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


using std::cout;
using std::endl;

ros::Publisher pub;

void publish_cropped_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

int main(int argc, char** argv)
{
    std::string sub_topic_name = "/kinect2/qhd/points";
    std::string pub_topic_name = "/aaron/cropped_cloud";
    ros::init (argc, argv, "crop_cloud");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(sub_topic_name, 10, publish_cropped_cloud);
    pub = nh.advertise<sensor_msgs::PointCloud2> (pub_topic_name , 1);
    ros::spin();
}


void publish_cropped_cloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cout << "got cloud" << endl;
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*cloud_msg, cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::PointCloud<pcl::PointXYZRGB> old_v_cloud;
    pcl::fromPCLPointCloud2(cloud, old_v_cloud);

    //-----------Downsampling------------------

    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(old_v_cloud));
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    //-----------Crop Box Filter---------------

    double minX = -0.5;
    double minY = -0.8;
    double minZ = 0;
    double maxX = .5;
    double maxY = 2;
    double maxZ = 5;

    pcl::PointCloud<pcl::PointXYZRGB> filtered_old_v_cloud;
    for(const auto& point : *cloud_filtered)
    {
        if(point.x >= minX && point.y >= minY && point.z >= minZ && point.x <= maxX && point.y <= maxY && point.z <= maxZ )
        {
            filtered_old_v_cloud.push_back(point);
        }
    }

    //-----------publish clustered point cloud-----------
    pcl::PCLPointCloud2 pub_cloud;
    pcl::toPCLPointCloud2(filtered_old_v_cloud, pub_cloud);
//    pcl::toPCLPointCloud2(plane_cloud, pub_cloud);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(pub_cloud, output);
    output.header.frame_id = "kinect2_ir_optical_frame";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}