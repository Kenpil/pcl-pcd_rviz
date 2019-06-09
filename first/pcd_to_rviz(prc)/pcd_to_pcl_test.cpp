#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char **argv)
{
    double x_cloud;
    double y_cloud;
    double z_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);
    pcl::VoxelGrid<pcl::PointXYZ> sor;

    std_msgs::String filemsg;
    ros::NodeHandle pn("~");
    std::string pcd_file_name_chatter = "xxx.pcd";
    pn.getParam("pcdfile", pcd_file_name_chatter);
    filemsg.data = pcd_file_name_chatter;
    float LeafSize;
    pn.getParam("downsampling_rate", LeafSize);
    ROS_INFO("LeafSize%f\n", LeafSize);
    pcl::PCDReader reader;
    reader.read(filemsg.data.c_str(), *cloud);

    sor.setInputCloud(cloud);
    sor.setLeafSize(LeafSize, LeafSize, LeafSize);
    //sor.setLeafSize(0.1, 0.1, 0.1);
    sor.filter(*cloud_filtered);

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "map";
    msg->height = cloud_filtered->height;
    msg->width = cloud_filtered->width;

    for (size_t i = 0; i < cloud_filtered->size(); i++)
    {
        x_cloud = cloud_filtered->points[i].x;
        y_cloud = cloud_filtered->points[i].y;
        z_cloud = cloud_filtered->points[i].z;
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