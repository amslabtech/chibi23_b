
#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

struct Node
{
    int    x = 0;
    int    y = 0;
    int    parent_x = 0;
    int    parent_y = 0;
    double g = 0.0;
    double f = 0.0;
};

struct Motion
{
    int dx;
    int dy;
    double cost;
};

class Astar
{
public:
    Astar();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);

    int hz_;

    bool   map_checker_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_path_;
    ros::Publisher pub_wp_path_;

    nav_msgs::OccupancyGrid map_;

};

