
#include "local_path_planner.h"

DWA::DWA():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("goal_tolerance", goal_tolerance_, {0.3});
    private_nh_.param("max_vel", max_vel_, {0.2});
    private_nh_.param("min_vel", min_vel_, {0.0});
    private_nh_.param("max_yawrate", max_yawrate_, {1.0});
    private_nh_.param("max_accel", max_accel_, {1.0});
    private_nh_.param("max_yawaccel", max_yawaccel_, {1.0});
    private_nh_.param("dt",dt,{0.5});
    private_nh_.param("predict_time", predict_time{1.0});
    private_nh_.param("weight_heading", weight_heading_, {1.0});
    private_nh_.param("weight_distance", weight_distance_, {1.0});
    private_nh_.param("weight_velocity", weight_velocity_, {1.0});
    private_nh_.param("search_range", search_range_, {5.0});
    private_nh_.param("roomba_radius", roomba_radius_, {0.2});
    private_nh_.param("radius_margin", radius_margin_, {0.2});
    private_nh_.param("vel_reso", vel_reso_, {0.01});
    private_nh_.param("yawrate_reso", yawrate_reso_, {0.01});

    //Subscriber
    sub_local_goal_ = nh_.subscribe("/local_goal", 1, &DWA::local_goal_callback, this);
    sub_ob_poses_   = nh_.subscribe("/local_map/obstacle", 1, &DWA::ob_poses_callback, this);

    //Publisher
    pub_cmd_speed_    = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
}

void DWA::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_goal=*msg;
}

void DWA::obstacle_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    ob_poses_ = *msg;
    flag_ob_poses_ = true;
}


bool DWA::can_move()
{
        if(!(flag_local_goal_ && flag_ob_poses_)) return false;


        const double dx = local_goal_.point.x;
        const double dy = local_goal_.point.y;
        const double dist_to_goal = hypot(dx, dy);


        if(dist_to_goal > goal_tolerance_)
            return true;
        else
            return false;
}

//Dynamic Window計算
void DWA::calc_dynamic_window()
{
        // 車両モデルによるWindow
    double Vs[] = {min_vel_, max_vel_, -max_yawrate_, max_yawrate_};

    double Vd[] = {roomba_.velocity - max_accel_*dt_,roomba_.velocity + max_accel_*dt_,roomba_.yawrate  - max_dyawrate_*dt_,roomba_.yawrate  + max_dyawrate_*dt_};

    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}


double DWA::calc_evaluation(const std::vector<State>& traj)
{

    const double heading_cost  = weight_heading_ * calc_heading_eval();
    const double distance_cost = weight_dist_    * calc_dist_eval();
    const double velocity_cost = weight_vel_     * calc_vel_eval();



    const double total_cost= heading_cost + distance_cost + velocity_cost;
    return total_cost;
}

double DWA::calc_heading_eval(const std::vector<State>& traj)
{
    const double theta = traj.back().yaw;

    const double goal_theta = atan2(local_goal_.point.y - traj.back().y, local_goal_.point.x - traj.back().x);

    double target_theta = 0.0;

    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;


    const double heading_eval = (M_PI - abs(regulate_angle(target_theta)))/M_PI;

    return heading_eval;
}
















