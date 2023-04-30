#include "local_map_creator/local_map_creator.h"

LocalMapCreator::LocalMapCreator():private_nh_("~")
{
    private_nh_.param("hz", hz_);
    private_nh_.param("map_size", map_size_);
    private_nh_.param("map_reso", map_reso_);

    sub_obs_poses_ = nh_.subscribe("/local_map/obstacle", 1, &LocalMapCreator::obs_poses_callback, this);

    pub_local_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_map", 1);

    local_map_.header.frame_id = "base_link";
    local_map_.info.resolution = map_reso_; //マップの解像度
    local_map_.info.width = int(round(map_size_/map_reso_)); //マップの幅
    local_map_.info.height = int(round(map_size_/map_reso_)); //マップの高さ
    local_map_.info.origin.position.x = -map_size_/2.0;
    local_map_.info.origin.position.y = -map_size_/2.0;

    local_map_.data.reserve(local_map_.info.width * local_map_.info.height);
}

void LocalMapCreator::obs_poses_callback(const geometry_msgs::PoseArray::ConstPtr& msg) //障害物の位置のコールバック
{
    obs_poses_ = *msg;
    is_obs_poses_checker_ = true;
}

void LocalMapCreator::init_map() //マップの初期化
{
    local_map_.data.clear(); //配列を空に

    const int size = local_map_.info.width * local_map_.info.height;
    for(int i=0; i<size; i++)
    {
        local_map_.data.push_back(-1); //すべてを未知
    }
}


void LocalMapCreator::update_map() //マップの更新
{
    init_map();

    for(const auto& obs_pose : obs_poses_.poses)
    {
        const double obs_x     = obs_pose.position.x;
        const double obs_y     = obs_pose.position.y;
        const double obs_dist  = hypot(obs_y, obs_x);
        const double obs_angle = atan2(obs_y, obs_x);

        for(double dist_from_start=0.0; (dist_from_start<obs_dist and is_map_checker(dist_from_start, obs_angle)); dist_from_start+=map_reso_)
        {
            const int grid_index = d_a_grid_index(dist_from_start, obs_angle);
            local_map_.data[grid_index] = 0; //「空き」にする
        }

        if(is_map_checker(obs_dist, obs_angle)) //マップ内に障害物がいる場合
        {
            const int grid_index = xy_grid_index(obs_x, obs_y);
            local_map_.data[grid_index] = 100; //「占有」にする
        }
    }


    pub_local_map_.publish(local_map_);

}

bool LocalMapCreator::is_map_checker(const double dist, const double angle) //マップ内にいるか判定
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    if(index_x<local_map_.info.width and index_y<local_map_.info.height) //インデックスがそれぞれマップに収まっているか
    {
        return true;
    }
    else
    {
        return false;
    }
}

int LocalMapCreator::d_a_grid_index(const double dist, const double angle) //距離と角度からグリッドに
{
    const double x = dist * cos(angle);
    const double y = dist * sin(angle);

    return xy_grid_index(x, y);
}

int LocalMapCreator::xy_grid_index(const double x, const double y) //座標からグリッドに
{
    const int index_x = int(round((x - local_map_.info.origin.position.x) / local_map_.info.resolution));
    const int index_y = int(round((y - local_map_.info.origin.position.y) / local_map_.info.resolution));

    return index_x + (index_y * local_map_.info.height);
}

void LocalMapCreator::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(is_obs_poses_checker_)
        {
            update_map();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "local_map_creator"); // ノードの初期化
    LocalMapCreator local_map_creator;
    local_map_creator.process();

    return 0;
}
