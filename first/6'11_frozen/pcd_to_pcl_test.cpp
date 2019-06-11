#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
//#include "visualization_msgs/Marker.h"
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>                 // 追記
#include <pcl/kdtree/kdtree.h>                 // 追記
#include <pcl/segmentation/extract_clusters.h> // 追記
//#include <rviz/default_plugin/interactive_markers/interactive_marker_control.h>
//#include <interactive_point_cloud.h>
//#include <interactive_markers/interactive_marker_server.h>
//#include <interactive_markers/menu_handler.h>


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char **argv)
{
    double x_cloud;
    double y_cloud;
    double z_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloud::Ptr cloud_passthrough_;
    ros::init(argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud>("points2", 1);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_;
    ros::Publisher pub_clusters_;

    std_msgs::String filemsg;


    ros::NodeHandle pn("~");
    //std::string pcd_file_name_chatter = "xxx.pcd";
    std::string pcd_file_name_chatter = "/home/kenshiro/imu_test_ws/table_scene_lms400.pcd";
    /*pn.getParam("pcdfile", pcd_file_name_chatter);
    filemsg.data = pcd_file_name_chatter;
    float LeafSize;
    pn.getParam("downsampling_rate", LeafSize);
    ROS_INFO("LeafSize%f\n", LeafSize);
    pcl::PCDReader reader;
    reader.read(filemsg.data.c_str(), *cloud);*/


    sor.setInputCloud(cloud);
    //sor.setLeafSize(LeafSize, LeafSize, LeafSize);
    sor.setLeafSize(0.1, 0.1, 0.1);
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

    std::vector<pcl::PointIndices> cluster_indices;
    tree_->setInputCloud(cloud_filtered);
    /*ec_.setInputCloud(cloud_filtered);
    ec_.extract(cluster_indices);
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    /*
    for (std::vector<pcl::PointIndices>::const_iterator
             it = cluster_indices.begin(),
             it_end = cluster_indices.end();
         it != it_end; ++it, ++marker_id)
    {
        Eigen::Vector4f min_pt, max_pt;
        pcl::getMinMax3D(*cloud_filtered, *it, min_pt, max_pt);
        Eigen::Vector4f cluster_size = max_pt - min_pt;
        if (cluster_size.x() > 0 && cluster_size.y() > 0 && cluster_size.z() > 0)
        {
           // marker_array.markers.push_back(
            //    makeMarker(
              //      msg->header.frame_id, "cluster", marker_id, min_pt, max_pt, 0.0f, 1.0f, 0.0f, 0.5f));
        }
    }
    if (marker_array.markers.empty() == false)
    {
        pub_clusters_.publish(marker_array);
    }
    ROS_INFO("points (src: %zu, paththrough: %zu, voxelgrid: %zu, cluster: %zu)",
             msg->size(), cloud_passthrough_->size(), cloud_filtered->size(),
             cluster_indices.size());
*/
    ros::Rate loop_rate(4);

    while (nh.ok())
    {
        //msg->header.stamp = ros::Time::now().toNSec();
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
}