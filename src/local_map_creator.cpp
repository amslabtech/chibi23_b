#include "obstacle_detector/obstacle_detector.h"

LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_size", map_size_, {4});
    private_nh_.param("map_reso", map_reso_, {0.02});
    private_nh_.param("roomba_radius", roomba_radius_, {0.2});
    private_nh_.param("flag_map_view", flag_map_view_, {true});
    // private_nh_.param("flag_map_view", flag_map_view_, {false});
    // private_nh_.param("flag_pose_callback", flag_pose_callback_, {true});
    private_nh_.param("flag_pose_callback", flag_pose_callback_, {false});
    // private_nh_.param("flag_odo_callback", flag_odo_callback_, {true});
    private_nh_.param("flag_odo_callback", flag_odo_callback_, {false});

    laser_sub_ = nh_.subscribe("scan", 10, &LocalMapCreator::laser_callback, this);
    pose_sub_ = nh_.subscribe("/estimated_pose", 1, &LocalMapCreator::pose_callback, this);
    odo_sub_ = nh_.subscribe("/roomba/odometry", 100, &LocalMapCreator::odo_callback, this);
    local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map_test", 10);
    obstacle_poses_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 10);

    obstacle_poses_.header.frame_id = "base_link";
    local_map_.header.frame_id = "base_link";
    local_map_.info.resolution = map_reso_;
    local_map_.info.width = map_size_ / map_reso_;
    local_map_.info.height = map_size_ / map_reso_;
    local_map_.info.origin.position.x = - map_size_ / 2;
    local_map_.info.origin.position.y = - map_size_ / 2;
    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
}

//laserから情報を受け取る
void LocalMapCreator::laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    laser_ = *msg;
    if(flag_map_view_)
    {
        init_map();
    }
    create_local_map();
    is_laser_checker_ = true;
}

//現在位置とマップのずれをチェック
void LocalMapCreator::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if(flag_pose_callback_)
    {
        current_pose_ = *msg;
        diff_.position.x = current_pose_.pose.position.x - previous_pose_.position.x;
        diff_.position.y = current_pose_.pose.position.y - previous_pose_.position.y;
        previous_pose_ = current_pose_.pose;

        if(is_first_pose_checker_)
        {
            is_second_pose_checker_ = true;
        }
        is_first_pose_checker_ = true;
    }
    else
    {
        return;
    }
}

void LocalMapCreator::odo_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    if(flag_odo_callback_)
    {
        current_odo_ = *msg;
        diff_.position.x = current_odo_.pose.pose.position.x - previous_odo_.pose.pose.position.x;
        diff_.position.y = current_odo_.pose.pose.position.y - previous_odo_.pose.pose.position.y;
        previous_odo_ = current_odo_;

        if(is_first_pose_checker_)
        {
            is_second_pose_checker_ = true;
        }
        is_first_pose_checker_ = true;
    }
    else
    {
        return;
    }
}

//マップの初期化
alMapCreator::init_map()
{
    local_map_.data.clear();
    int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
    {
        local_map_.data.push_back(-1);
    }
}

//受け取ったx,yについて、マップのどのセルを示しているか計算する
int LocalMapCreator::xy_to_map_index(double x, double y)
{
    int x_index = (x - local_map_.info.origin.position.x) / map_reso_;
    int y_index = (y - local_map_.info.origin.position.y) / map_reso_;

    return x_index + y_index * local_map_.info.width;
}

//受け取ったx,yが、あらかじめ決めたlocal_mapのサイズにおさまっているか判定
bool LocalMapCreator::is_map_range_checker(double x, double y)
{
    double x_min = local_map_.info.origin.position.x;
    double y_min = local_map_.info.origin.position.y;
    double x_max = x_min + local_map_.info.width * local_map_.info.resolution;
    double y_max = y_min + local_map_.info.height * local_map_.info.resolution;

    if((x_min < x) && (x_max > x) && (y_min < y) && (y_max > y))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//受け取った角度が、車体の柱の部分かどうか判定
bool LocalMapCreator::is_ignore_angle_checker(double angle)
{
    angle = abs(angle);

    if((angle > M_PI * 2/16) && (angle < M_PI * 5/16))  //柱の位置
    {
        return false;
    }
    else if(angle > M_PI * 11/16)                       //柱の位置
    {
        return false;
    }
    else                                                //柱の位置ではない
    {
        return true;
    }
}

//laserの測定値がroomba半径付近かどうか判定
bool LocalMapCreator::is_range_checker(double laser_range)
{
    if(laser_range < roomba_radius_)
    {
        return false;
    }
    else
    {
        return true;
    }
}

//角度ごとにマップを作成
void LocalMapCreator::create_line(double angle, double laser_range)
{
    if(!is_ignore_angle_checker(angle))
    {
        laser_range = map_size_;
    }

    for(double distance = 0; distance < map_size_; distance+=map_reso_)
    {
        double x_now = distance * std::cos(angle);
        double y_now = distance * std::sin(angle);

        if(!is_map_range_checker(x_now, y_now))
        {
            return;
        }

        int map_index = xy_to_map_index(x_now, y_now);

        if(first_map_checker_)  //マップ作成2回目以降

