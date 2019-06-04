#include <iostream>
#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <std_msgs/String.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char **argv)
{
    double x_cloud;
    double y_cloud;
    double z_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);

    std_msgs::String filemsg;
    ros::NodeHandle pn("~");
    std::string pcd_file_name_chatter = "xxx.pcd";
    pn.getParam("pcdfile", pcd_file_name_chatter);
    filemsg.data = pcd_file_name_chatter;
    //pcl::io::loadPCDFile<pcl::PointXYZ>("/home/kenshiro/imu_test_ws/table_scene_lms400.pcd", *cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>(filemsg.data.c_str(), *cloud);

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "map";
    msg->height = cloud->height;
    msg->width = cloud->width;

    for (size_t i = 0; i < cloud->size(); i++)
    {
        x_cloud = cloud->points[i].x;
        y_cloud = cloud->points[i].y;
        z_cloud = cloud->points[i].z;
        msg->points.push_back(pcl::PointXYZ(x_cloud, y_cloud, z_cloud));
    }

    ros::Rate loop_rate(4);

    while (nh.ok())
    {
        //msg->header.stamp = ros::Time::now().toNSec();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
