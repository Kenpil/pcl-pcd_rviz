#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h> //SAC_SAMPLE_SIZE?
#include <pcl/segmentation/sac_segmentation.h> //SAC_SAMPLE_SIZE?
#include <pcl/segmentation/extract_clusters.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace std;

int main(int argc, char **argv)
{
    double x_cloud;
    double y_cloud;
    double z_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
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


    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.02);

    int i = 0, nr_points = (int)cloud_filtered->points.size();
    while (cloud_filtered->points.size() > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(cloud_filtered);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_filtered);
        extract.setIndices(inliers);
        extract.setNegative(false);

        // Get the points associated with the planar surface
        extract.filter(*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative(true);
        extract.filter(*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02); // 2cm
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(25000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_cluster, false); //*
        j++;
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
