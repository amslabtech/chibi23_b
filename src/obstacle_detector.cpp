#include "obstacle_detector/obstacle_detector.h"

ObstacleDetector::ObstacleDetector():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("ignore_dist", ignore_dist_);

    obs_poses_.header.frame_id = "base_link";

    laser_sub_ = nh_.subscribe("/scan", 10, &ObstacleDetector::laser_callback, this);

    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 1);
}

//レーザーの値のコールバック
void ObstacleDetector::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    is_laser_checker = true;
}

//障害物の検知
void ObstacleDetector::scan_obstacle()
{
    obs_poses_.poses.clear();

    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        const double angle = i * laser_.angle_increment + laser_.angle_min; //角度
        double dist  = laser_.ranges[i]; //直線距離

        //レーザーの外れ値を無視
        if(dist <= ignore_dist_)
        {
            continue;
        }

        //柱だった場合障害物としては認識しないようにする
        if(is_ignore_angle(angle))
        {
            continue;
        }

        //障害物の座標を代入
        geometry_msgs::Pose obs_pose;
        obs_pose.position.x = dist * cos(angle);
        obs_pose.position.y = dist * sin(angle);


        obs_poses_.poses.push_back(obs_pose);
    }
    pub_obs_poses_.publish(obs_poses_);
}

//ルンバの柱の角度による処理
bool ObstacleDetector::is_ignore_angle(double angle)
{
    angle = abs(angle); //絶対値

    //柱の角度の領域の場合、trueを返す
    if((angle > M_PI * 3/16) && (angle < M_PI * 5/16))
    {
        return true;
    }
    else if(angle > M_PI * 11/16)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//レーザーから値を受けとれていれば障害物の検知の実行
void ObstacleDetector::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(is_laser_checker)
        {
            scan_obstacle();
        }
        ros::spinOnce();
        loop_rate.sleep();

    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "obstacle_detector"); // ノードの初期化
    ObstacleDetector obstacle_detector;
    obstacle_detector.process();

    return 0;
}
