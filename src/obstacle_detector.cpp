#include "obstacle_detector/obstacle_detector.h"

ObstacleDetector::ObstacleDetector():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("laser_step", laser_step_, {3});
    private_nh_.param("ignore_dist", ignore_dist_, {0.01});
    private_nh_.getParam("ignore_angle_range_list", ignore_angle_range_list_);

    obs_poses_.header.frame_id = "base_link";

    laser_sub_ = nh_.subscribe("/scan", 10, &ObstacleDetector::laser_callback, this);

    pub_obs_poses_ = nh_.advertise<geometry_msgs::PoseArray>("/local_map/obstacle", 1);
}

void ObstacleDetector::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    is_laser_checker = true;
}

void ObstacleDetector::scan_obstacle()
{
    obs_poses_.poses.clear();

    for(int i=0; i<laser_.ranges.size(); i+=laser_step_)
    {
        const double angle = i * laser_.angle_increment + laser_.angle_min;
        double dist  = laser_.ranges[i];

        int index_incr = i;
        int index_decr = i;
        while(dist <= ignore_dist_)
        {
            if(index_incr++ < laser_.ranges.size())
                dist = laser_.ranges[index_incr];
            if(dist<=ignore_dist_ and 0<=index_decr--)
                dist = laser_.ranges[index_decr];
        }

        if(is_ignore_angle(angle)) continue;

        geometry_msgs::Pose obs_pose;
        obs_pose.position.x = dist * cos(angle);
        obs_pose.position.y = dist * sin(angle);


        obs_poses_.poses.push_back(obs_pose);
    }
    pub_obs_poses_.publish(obs_poses_);
}

bool ObstacleDetector::is_ignore_angle(double angle)
{
    angle = abs(angle);
    const int size = ignore_angle_range_list_.size();

    for(int i=0; i<size/2; i++)
    {
        if(ignore_angle_range_list_[i*2] < angle and angle < ignore_angle_range_list_[i*2 + 1])
            return true;
    }

    if(size%2 == 1)
    {
        if(ignore_angle_range_list_[size-1] < angle)
            return true;
    }

    return false;
}

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
