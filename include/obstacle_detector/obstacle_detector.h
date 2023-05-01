#ifndef OBSTACLE_DETECTOR_H
#define OBSTACLE_DETECTOR_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <math.h>
#include <iostream>

class ObstacleDetector
{
    public:
        ObstacleDetector();
        void process();

    private:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
        void scan_obstacle();
        bool is_ignore_angle(double angle);

        int    hz_;
        int    laser_step_;
        double ignore_dist_;
        std::string robot_frame_;
        std::vector<double> ignore_angle_range_list_;

        bool is_laser_checker = false;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber laser_sub_;
        ros::Publisher pub_obs_poses_;
        geometry_msgs::PoseArray obs_poses_;
        sensor_msgs::LaserScan   laser_;
};

#endif

