/*
00002  * Software License Agreement (BSD License)
00003  *
00004  *  Copyright (c) 2009, Willow Garage, Inc.
00005  *  All rights reserved.
00006  *
00007  *  Redistribution and use in source and binary forms, with or without
00008  *  modification, are permitted provided that the following conditions
00009  *  are met:
00010  *
00011  *   * Redistributions of source code must retain the above copyright
00012  *     notice, this list of conditions and the following disclaimer.
00013  *   * Redistributions in binary form must reproduce the above
00014  *     copyright notice, this list of conditions and the following
00015  *     disclaimer in the documentation and/or other materials provided
00016  *     with the distribution.
00017  *   * Neither the name of Willow Garage, Inc. nor the names of its
00018  *     contributors may be used to endorse or promote products derived
00019  *     from this software without specific prior written permission.
00020  *
00021  *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
00022  *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
00023  *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
00024  *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
00025  *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
00026  *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
00027  *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
00028  *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
00029  *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
00030  *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
00031  *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
00032  *  POSSIBILITY OF SUCH DAMAGE.
00033  *
00034  * $Id: pcd_to_pointcloud.cpp 33238 2010-03-11 00:46:58Z rusu $
00035  *
00036  */

// ROS core
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "pcl_ros/publisher.h"

using namespace std;

class PCDGenerator
{
  protected:
    string tf_frame_;
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

  public:
    // ROS messages
    sensor_msgs::PointCloud2 cloud_;

    string file_name_, cloud_topic_;
    double wait_;

    pcl_ros::Publisher<sensor_msgs::PointCloud2> pub_;

    PCDGenerator() : tf_frame_("/base_link"), private_nh_("~")
    {
        // Maximum number of outgoing messages to be queued for delivery to subscribers = 1

        cloud_topic_ = "cloud_pcd";
        pub_.advertise(nh_, cloud_topic_.c_str(), 1);
        private_nh_.param("frame_id", tf_frame_, std::string("/base_link"));
        ROS_INFO("Publishing data on topic %s with frame_id %s.", nh_.resolveName(cloud_topic_).c_str(), tf_frame_.c_str());
    }

    // Start
    int
    start()
    {
        if (file_name_ == "" || pcl::io::loadPCDFile(file_name_, cloud_) == -1)
            return (-1);
        cloud_.header.frame_id = tf_frame_;
        return (0);
    }

    // Spin (!)
    bool spin()
    {
        int nr_points = cloud_.width * cloud_.height;
        string fields_list = pcl::getFieldsList(cloud_);
        double interval = wait_ * 1e+6;
        while (nh_.ok())
        {
            ROS_DEBUG_ONCE("Publishing data with %d points (%s) on topic %s in frame %s.", nr_points, fields_list.c_str(), nh_.resolveName(cloud_topic_).c_str(), cloud_.header.frame_id.c_str());
            cloud_.header.stamp = ros::Time::now();

            if (pub_.getNumSubscribers() > 0)
            {
                ROS_DEBUG("Publishing data to %d subscribers.", pub_.getNumSubscribers());
                pub_.publish(cloud_);
            }
            else
            {
                // check once a second if there is any subscriber
                ros::Duration(1).sleep();
                continue;
            }

            usleep(interval);

            if (interval == 0) // We only publish once if a 0 seconds interval is given
            {
                // Give subscribers 3 seconds until point cloud decays... a little ugly!
                ros::Duration(3.0).sleep();
                break;
            }
        }
        return (true);
    }
};

/* ---[ */
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Syntax is: " << argv[0] << " <file.pcd> [publishing_interval (in seconds)]" << std::endl;
        return (-1);
    }

    ros::init(argc, argv, "pcd_to_pointcloud");

    PCDGenerator c;
    c.file_name_ = string(argv[1]);
    // check if publishing interval is given
    if (argc == 2)
    {
        c.wait_ = 0;
    }
    else
    {
        c.wait_ = atof(argv[2]);
    }

    if (c.start() == -1)
    {
        ROS_ERROR("Could not load file %s. Exiting.", argv[1]);
        return (-1);
    }
    ROS_INFO("Loaded a point cloud with %d points (total size is %zu) and the following channels: %s.", c.cloud_.width * c.cloud_.height, c.cloud_.data.size(), pcl::getFieldsList(c.cloud_).c_str());
    c.spin();
    return (0);
}
/* ]--- */
