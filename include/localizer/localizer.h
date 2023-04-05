#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//構造体
struct State
{
    double x;
    double y;
    double yaw;
    double velocity;
    double yawrate;
};

struct

class DWA
{
    public:
        DWA();
        void process();

    private:
        //コールバック関数
        void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void obstacle_poses_callback(const geometry_msgs::PoseArray::ConstPtr &msg);

        regulate_angle(double yaw);
        std::vector<double> calc_dynamic_window();
        void   visualize_traj(const std::vector<State>& traj, const ros::Publisher& pub_local_path, ros::Time now);
        double calc_evaluation(std::vector<State> &trajectory);
        double calc_heading_score(std::vector<State> &trajectory);
        double calc_dist_score(std::vector<State> &trajectory);
        double calc_velo_score(std::vector<State> &trajectory);
        void roomba_control(double velocity, double yawrate);
        void visualize_trajectory(std::vector<State> &trajectory, ros::Publisher &publisher);

        bool can_move();
        std::vector<double> calc_input();


        int hz_;
        int mode_;
        double max_vel_;
        double min_vel_;
        double max_yawrate_;
        double max_accel_;
        double max_dyawrate_;
        double dt_;
        double roomba_radius_;
        double radius_margin_;
        double goal_tolerance_;
        double search_range_;
        double predict_time_;
        double yawrate_reso_;
        double vel_reso_;


        bool flag_local_goal_ = false;
        bool flag_ob_poses_   = false;


        double weight_heading_;
        double weight_distance_;
        double weight_velocity_;


        State roomba_;
        DynamicWindow dw_;

        //NodeHandle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        //Subscriber
        ros::Subscriber sub_local_goal_;
        ros::Subscriber sub_ob_poses_;

        //Publisher
        ros::Publisher pub_cmd_speed_;
        ros::Publisher pub_optimal_path_;
        ros::Publisher pub_predict_path_;

        geometry_msgs::PointStamped local_goal_;
        geometry_msgs::PoseArray    ob_poses_;

        roomba_500driver_meiji::RoombaCtrl cmd_velocity_;
};

#endif


