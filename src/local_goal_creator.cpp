#include "local_goal_creator/local_goal_creator.h"

LocalGoalCreator::LocalGoalCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("goal_index", goal_index_, {50});
    private_nh_.param("local_goal_dist", local_goal_dist_, {1});

    global_path_sub_ = nh_.subscribe("/global_path", 1, &LocalGoalCreator::global_path_callback, this);
    current_pose_sub_ = nh_.subscribe("/estimated_pose", 10, &LocalGoalCreator::current_pose_callback, this);

    local_goal_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal", 10);

    local_goal_.header.frame_id = "map";
}

//A*から経路の座標をもらう
void LocalGoalCreator::global_path_callback(const nav_msgs::Path::ConstPtr &msg)
{
    global_path_ = *msg;
    local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;  //ゴールにxの値を代入
    local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;  //ごーるにyの値を代入
    is_global_path_checker_ = true;                                        //値を受け取ったことを確認する
}

//現在位置をもらう
void LocalGoalCreator::current_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose_ = *msg;
    is_current_pose_checker_ = true;  //値を受け取ったことを確認する
}

//local_goalを作成する
void LocalGoalCreator::make_local_goal()
{
    double dx = current_pose_.pose.position.x - global_path_.poses[goal_index_].pose.position.x;  //A*でもらったxと現在のxの差
    double dy = current_pose_.pose.position.y - global_path_.poses[goal_index_].pose.position.y;  //A*でもらったyと現在のyの差
    double distance = hypot(dx, dy);                                                              //直線距離の差を求める

    if(distance < local_goal_dist_) // 設定したゴールの値の範囲内に入っていれば
    {
        goal_index_ += 3;  //goal位置を、callback関数で受け取った時よりも少し先へ移動させる

        if(goal_index_ < global_path_.poses.size())
        {
            local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
            local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
        }
        else
        {
            goal_index_ = global_path_.poses.size() -1;
            local_goal_.point.x = global_path_.poses[goal_index_].pose.position.x;
            local_goal_.point.y = global_path_.poses[goal_index_].pose.position.y;
        }
    }
}

void LocalGoalCreator::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok()) {
        if(is_global_path_checker_ && is_current_pose_checker_)
        {
            make_local_goal();
            local_goal_.header.stamp = ros::Time::now();
            local_goal_pub_.publish(local_goal_);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoalCreator localgoalcreator;
    localgoalcreator.process();

    return 0;
}

