#include "global_path_planner/global_path_planner.h"

Astar::Astar():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_checker", map_checker_, {false});
//    private_nh_.param("path_checker", path_checker_, {false});
//    private_nh_.param("is_reached", is_reached_, {false});
//    private_nh_.param("wall_cost", wall_cost_, {1e10});

    sub_map_ = nh_.subscribe("/new_map", 10, &Astar::map_callback, this);                   //mapデータの受信
    pub_path_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);                           //出力するパス
    pub_wp_path_ = nh_.advertise<nav_msgs::Path>("/wp_path", 1);
}


void Astar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    map_checker_ = true;
}

void process()
{

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planner");
    Astar astar;
    astar.process();
    return 0;
}


