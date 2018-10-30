//
// Created by aaron on 28/09/18.
//

#include <iostream>
#include <memory>
#include <ros/ros.h>
#include "object_tracker.hpp"

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

using std::cout;
using std::endl;

ros::Publisher pub;

void track_objects(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

int main(int argc, char** argv)
{
    std::string sub_topic_name = "/kinect2/qhd/points";
    std::string pub_topic_name = "/aaron/plane";
    ros::init (argc, argv, "pcl_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe(sub_topic_name, 10, track_objects);
    pub = nh.advertise<sensor_msgs::PointCloud2> (pub_topic_name , 1);
    ros::spin();
}

void track_objects(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    cout << "got cloud" << endl;
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*cloud_msg, cloud);
    cout << "Num points: " << cloud.data.size() << endl;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.025);
    pcl::PointCloud<pcl::PointXYZ> old_v_cloud;
    pcl::fromPCLPointCloud2(cloud, old_v_cloud);

    //-----------Downsampling------------------

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(old_v_cloud));
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl; //*

    //-----------Crop Box Filter---------------

    double minX = -0.3;
    double minY = 0;
    double minZ = 0;
    double maxX = .25;
    double maxY = 2;
    double maxZ = 2;

    pcl::PointCloud<pcl::PointXYZ> filtered_old_v_cloud;
    for(const auto& point : *cloud_filtered)
    {
        if(point.x >= minX && point.y >= minY && point.z >= minZ && point.x <= maxX && point.y <= maxY && point.z <= maxZ )
        {
            filtered_old_v_cloud.push_back(point);
        }
    }


    //-----------------------------------------


    seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_old_v_cloud));
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }
//    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
//              << coefficients->values[1] << " "
//              << coefficients->values[2] << " "
//              << coefficients->values[3] << std::endl;

//    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

//    pcl::PointCloud<pcl::PointXYZ> plane_cloud;
//    for(ulong i = 0; i < inliers->indices.size(); ++i)
//    {
//        if(inliers->indices.at(i) < filtered_old_v_cloud.points.size() && inliers->indices.at(i) > 0) {
//            plane_cloud.push_back(filtered_old_v_cloud.points.at(static_cast<ulong>(inliers->indices.at(i))));
//        }
//    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
    extract.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_old_v_cloud));
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(above_surface_indices->indices);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_old_v_cloud));

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(filtered_old_v_cloud));
    ec.setIndices(above_surface_indices);
    ec.setClusterTolerance (0.03); // 2cm
    ec.setMinClusterSize (150);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.extract (cluster_indices);
    pcl::PointCloud<pcl::PointXYZRGB> cluster_cloud;
    std::cout << "Num CLusters: " << cluster_indices.size() << std::endl;
    for(ulong i = 0; i < cluster_indices.size(); ++i)
    {
        for(const auto& index: cluster_indices.at(i).indices)
        {
            pcl::PointXYZRGB point;
            point.x = filtered_old_v_cloud.at(index).x;
            point.y = filtered_old_v_cloud.at(index).y;
            point.z = filtered_old_v_cloud.at(index).z;
            point.a = 255;
            if(i%3 == 0)
            {
                point.r = 255;
                point.g = 0;
                point.b = 0;
            }
            else if(i %3 == 1)
            {
                point.r = 0;
                point.g = 255;
                point.b = 0;
            }
            else if(i %3 == 0)
            {
                point.r = 0;
                point.g = 97;
                point.b = 255;
            }

            cluster_cloud.push_back(point);
        }
    }

    pcl::PCLPointCloud2 pub_cloud;
    pcl::toPCLPointCloud2(cluster_cloud, pub_cloud);
//    pcl::toPCLPointCloud2(plane_cloud, pub_cloud);
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(pub_cloud, output);
    output.header.frame_id = "kinect2_rgb_optical_frame";
    output.header.stamp = ros::Time::now();
    pub.publish(output);
}