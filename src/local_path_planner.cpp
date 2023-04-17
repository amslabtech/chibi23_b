
#include "local_path_planner/local_path_planner.h"
DWA::DWA():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("goal_tolerance", goal_tolerance_, {0.3});
    private_nh_.param("max_vel", max_vel_, {0.2});
    private_nh_.param("min_vel", min_vel_, {0.0});
    private_nh_.param("max_yawrate", max_yawrate_, {1.0});
    private_nh_.param("max_accel", max_accel_, {1.0});
    private_nh_.param("max_yawaccel", max_dyawrate_, {1.0});
    private_nh_.param("dt",dt_,{0.5});
    private_nh_.param("predict_time", predict_time_, {1.0});
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
    sub_ob_poses_   = nh_.subscribe("/local_map/obstacle", 1, &DWA::obstacle_poses_callback, this);

    //Publisher
    pub_cmd_vel_    = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
    pub_predict_path_ = nh_.advertise<nav_msgs::Path>("/predict_local_paths", 1);
    pub_optimal_path_ = nh_.advertise<nav_msgs::Path>("/optimal_local_path", 1);
}

void DWA::local_goal_callback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    local_goal_=*msg;
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
    double Vs[] = {min_vel_,
                   max_vel_,
                   -max_yawrate_,
                   max_yawrate_};

    double Vd[] = {roomba_.velocity - max_accel_*dt_,
                   roomba_.velocity + max_accel_*dt_,
                   roomba_.yawrate  - max_dyawrate_*dt_,
                   roomba_.yawrate  + max_dyawrate_*dt_};

    dw_.min_vel     = std::max(Vs[0], Vd[0]);
    dw_.max_vel     = std::min(Vs[1], Vd[1]);
    dw_.min_yawrate = std::max(Vs[2], Vd[2]);
    dw_.max_yawrate = std::min(Vs[3], Vd[3]);
}


double DWA::calc_evaluation(const std::vector<State>& trajectory)
{
    const double heading_cost  = weight_heading_ * calc_heading_score(trajectory);
    const double distance_cost = weight_distance_ * calc_dist_score(trajectory);
    const double velocity_cost = weight_velocity_ * calc_vel_score(trajectory);



    const double total_cost= heading_cost + distance_cost + velocity_cost;
    return total_cost;
}

double DWA::calc_heading_score(const std::vector<State>& trajectory)
{
    const double theta = trajectory.back().yaw;

    const double goal_theta = atan2(local_goal_.point.y - trajectory.back().y, local_goal_.point.x - trajectory.back().x);

    double target_theta = 0.0;

    if(goal_theta > theta)
        target_theta = goal_theta - theta;
    else
        target_theta = theta - goal_theta;


    const double heading_eval = (M_PI - abs(regulate_angle(target_theta)))/M_PI;

    return heading_eval;
}

double DWA::calc_dist_score(const std::vector<State>& trajectory)
{
    double min_dist = search_range_;
    for(const auto& state : trajectory)
    {
        for(const auto& ob_pose : ob_poses_.poses)
        {
            const double dx   = ob_pose.position.x - state.x;
            const double dy   = ob_pose.position.y - state.y;
            const double dist = hypot(dx, dy);

            if(dist <= roomba_radius_+radius_margin_)
                                return -1e6;

            if(dist < min_dist)
                                min_dist = dist;
        }
    }
    return min_dist/search_range_;
}


//verocity評価関数
double DWA::calc_vel_score(const std::vector<State>& trajectory)
{
    if(0.0 < trajectory.back().velocity) // 前進
        return trajectory.back().velocity/max_vel_; // 正規化

    else
        return 0.0;
}

double DWA::regulate_angle(double angle)
{
    if(angle > M_PI)
        angle -= 2.0 * M_PI;
    if(angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

std::vector<double> DWA::calc_input()
{
    std::vector<double> input{0.0, 0.0};
    std::vector<std::vector<State>> trajectories;


    calc_dynamic_window();

    double max_score = -1000.0;
    int max_score_index = 0;


    int i=0;
    for(double velocity=dw_.min_vel; velocity<=dw_.max_vel; velocity+=vel_reso_)
    {
        for(double yawrate=dw_.min_yawrate; yawrate<=dw_.max_yawrate; yawrate+=yawrate_reso_)
        {
            std::vector<State> trajectory = calc_trajectory(velocity, yawrate); // 予測軌跡の生成

            const double score = calc_evaluation(trajectory);
            trajectories.push_back(trajectory);

            if(max_score < score)
            {
                max_score = score;
                input[0]  = velocity;
                input[1]  = yawrate;
                max_score_index = i;
            }
            i++;
        }
    }

    roomba_.velocity = input[0];
    roomba_.yawrate  = input[1];

    if(is_visible_)
    {
        ros::Time now = ros::Time::now();
        for(i=0; i<trajectories.size(); i++)
        {
            if(i == max_score_index)
                visualize_traj(trajectories[i], pub_optimal_path_, now);

            else
                visualize_traj(trajectories[i], pub_predict_path_, now);

        }
    }

    return input;
}

std::vector<State> DWA::calc_trajectory(const double velocity, const double yawrate)
{
    std::vector<State> trajectory;
    State state = {0.0, 0.0, 0.0, 0.0, 0.0};

    for(double t=0.0; t<=predict_time_; t+=dt_)
    {
        move(state, velocity, yawrate);
        trajectory.push_back(state);
    }

    return trajectory;
}

void DWA::move(State& state, const double velocity, const double yawrate)
{
    state.yaw      += yawrate * dt_;
    state.yaw       = regulate_angle(state.yaw);
    state.x        += velocity * cos(state.yaw) * dt_;
    state.y        += velocity * sin(state.yaw) * dt_;
    state.velocity  = velocity;
    state.yawrate   = yawrate;
}

void DWA::roomba_control(double velocity, double yawrate)
{
        cmd_velocity_.mode = 11;  //mode11:速度と角速度を設定
        cmd_velocity_.cntl.linear.x = velocity;
        cmd_velocity_.cntl.angular.z = yawrate;

        pub_cmd_vel_.publish(cmd_velocity_);
}

void DWA::visualize_traj(const std::vector<State>& trajectory, const ros::Publisher& pub_local_path, ros::Time now)
{
    nav_msgs::Path local_path;
    local_path.header.stamp = now;
    local_path.header.frame_id = "base_link";

    geometry_msgs::PoseStamped pose;
    pose.header.stamp = now;
    pose.header.frame_id = "base_link";


    for(const auto& state : trajectory)
    {
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        local_path.poses.push_back(pose);
    }
    pub_local_path.publish(local_path);
}

void DWA::process()
{
    ros::Rate loop_rate(hz_); // 制御周波数の設定
    tf2_ros::TransformListener tf_listener(tf_buffer_);

    while(ros::ok())
    {
        if(can_move())
        {
            const std::vector<double> input = calc_input();
            roomba_control(input[0], input[1]);
        }
        else
        {
            roomba_control(0.0, 0.0);
        }
        ros::spinOnce();   // コールバック関数の実行
        loop_rate.sleep(); // 周期が終わるまで待つ
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_path_planner"); // ノードの初期化
    DWA dwa;
    dwa.process();

    return 0;
}


