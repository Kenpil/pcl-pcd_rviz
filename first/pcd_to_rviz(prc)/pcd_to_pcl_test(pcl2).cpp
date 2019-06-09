#include <iostream>
#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <std_msgs/String.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char **argv)
{
    ROS_INFO("I started\n");
    double x_cloud;
    double y_cloud;
    double z_cloud;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    std_msgs::String filemsg;
    ros::NodeHandle pn("~");
    std::string pcd_file_name_chatter = "xxx.pcd";
    pn.getParam("pcdfile", pcd_file_name_chatter);
    filemsg.data = pcd_file_name_chatter;
    float LeafSize;
    pn.getParam("downsampling_rate", LeafSize);
    ROS_INFO("LeafSize%f\n", LeafSize);

    //pcl::io::loadPCDFile<pcl::PointXYZ>(filemsg.data.c_str(), *cloud);
    pcl::PCDReader reader;
    reader.read(filemsg.data.c_str(), *cloud);

    PointCloud::Ptr msg (new PointCloud);
    //msg->header.frame_id = "map";
    //msg->height = cloud_filtered->height;
    //msg->width = cloud_filtered->width;

    sor.setInputCloud(cloud);
    sor.setLeafSize(LeafSize, LeafSize, LeafSize);
    //sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud_filtered);

    /*for (size_t i = 0; i < cloud->size(); i++)
    {
        x_cloud = cloud_filtered->points[i].x;
        y_cloud = cloud_filtered->points[i].y;
        z_cloud = cloud_filtered->points[i].z;
        msg->points.push_back(pcl::PointXYZ(x_cloud, y_cloud, z_cloud));
    }*/

    ros::Rate loop_rate(4);

    while (nh.ok())
    {
        //msg->header.stamp = ros::Time::now().toNSec();
        //pub.publish(msg);
        //ROS_INFO("I got\n");
        //ROS_INFO("LeafSize%f\n", LeafSize);
        cloud_filtered->header.frame_id = "map";
        pub.publish(cloud_filtered);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
