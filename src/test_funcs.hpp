
#include <string>

#include <ros/ros.h>
#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class TEST_FUNCS
{
public:
    TEST_FUNCS(ros::NodeHandle& nh, const std::string& sub_topic_name, const std::string& pub_topic_name = "")
    {
        sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >(sub_topic_name, 10,&TEST_FUNCS::callback, this);
    }
    void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& input)
    {

    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;
};