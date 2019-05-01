#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

int main(int argc, char** argv)
{
    ros::init (argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

    ros::Rate loop_rate(1);
    while (nh.ok())
    {
        PointCloud::Ptr msg (new PointCloud);
        msg->header.frame_id = "map";
        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        msg->height = 480;
        msg->width = 640;
        msg->points.resize(480*640);


        for(int n=0; n<480; ++n) {
            for(int m=0; m<640; ++m) {
                msg->points[n*480+m].x = 1.0;
                msg->points[n*480+m].y = 0.01*(m-320);
                msg->points[n*480+m].z = 0.01*n;
                msg->points[n*480+m].r = 200;
                msg->points[n*480+m].g = 0;
                msg->points[n*480+m].b = 0;
                msg->points.push_back (msg->points[n*480+m]);
            }
        }

        pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
        pub.publish (msg);
        ros::spinOnce ();
        loop_rate.sleep ();
    }
}
