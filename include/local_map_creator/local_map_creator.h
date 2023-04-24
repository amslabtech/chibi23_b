#ifndef obstacle_detector_H
#define obstacle_detector_H

#include "ros/ros.h"
#include "tf2/utils.h"
#include "roomba_500driver_meiji/RoombaCtrl.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h" //グリッドマップ
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"

class LocalMapCreator
{
    public:
        LocalMapCreator();
        void process();

    private:
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr &msg);
        void obs_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg); //障害物の位置のコールバック

        int d_a_grid_index(const double dist, const double angle); //距離と角度をインデックスにする
        int xy_grid_index(const double x, const double y); //座標からインデックスに
        bool is_map_checker(const double dist, const double angle); //マップ内にいるかどうか判定

        void init_map(); //マップの初期化
        void update_map(); //マップの更新
        int hz_;
        double map_size_; //マップの一辺のサイズ
        double map_reso_; //マップの解像度
        bool is_obs_poses_checker_ = false; //コールバックのチェック

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        ros::Subscriber sub_obs_poses_;

        ros::Publisher pub_local_map_;

        geometry_msgs::PoseArray obs_poses_;
        sensor_msgs::LaserScan laser_;
        nav_msgs::OccupancyGrid local_map_;
};

#endif
