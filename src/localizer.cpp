#include "localizer/localizer.h"

//正規分布に従う乱数を生成する
std::random_device seed_gen;
std::default_random_engine engine(seed_gen());

//コンストラクタ
Particle::Particle(double x, double y, double yaw, double weight)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    weight_ = weight;
}

//x,y,yawを定める
void Particle::set_pose(double x, double y, double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
}

void Particle::set_weight(double weight)
{
    weight_ = weight;
}



//コンストラクタ
Localizer::Localizer():private_nh_("~")
{
    private_nh_.getParam("hz", hz_);
    private_nh_.getParam("particle_num",particle_num_);
    private_nh_.getParam("init_x", init_x_);
    private_nh_.getParam("init_y", init_y_);
    private_nh_.getParam("init_yaw", init_yaw_);
    private_nh_.getParam("init_dev", init_dev_);
    private_nh_.getParam("move_distance_dev", move_distance_dev_);
    private_nh_.getParam("move_direction_dev", move_direction_dev_);
    private_nh_.getParam("move_rotation_dev", move_rotation_dev_);
    private_nh_.getParam("laser_step", laser_step_);
    private_nh_.getParam("laser_ignore_range", laser_ignore_range_);
    private_nh_.getParam("laser_distance_dev", laser_distance_dev_);
    private_nh_.getParam("alpha_th", alpha_th_);
    private_nh_.getParam("expansion_limit", expansion_limit_);
    private_nh_.getParam("expansion_reset_x_dev", expansion_reset_x_dev_);
    private_nh_.getParam("expansion_reset_y_dev", expansion_reset_y_dev_);
    private_nh_.getParam("expansion_reset_yaw_dev", expansion_reset_yaw_dev_);
    private_nh_.getParam("alpha_slow_th", alpha_slow_th_);
    private_nh_.getParam("alpha_fast_th", alpha_fast_th_);
    private_nh_.getParam("resampling_reset_x_dev", resampling_reset_x_dev_);
    private_nh_.getParam("resampling_reset_y_dev", resampling_reset_y_dev_);
    private_nh_.getParam("resampling_reset_yaw_dev", resampling_reset_yaw_dev_);

    sub_odometry_ = nh_.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 10, &Localizer::laser_callback, this);
    sub_map_ = nh_.subscribe("/map", 10, &Localizer::map_callback, this);

    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);
}

//odometry,map,laserのコールdistバック関数
void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    if(!odometry_callback_flag_)
    {
        previous_odometry_ = *msg;
        odometry_callback_flag_ = true;
    }
    else
    {
        previous_odometry_ = current_odometry_;
    }

    if(moving_count_ < 1)
    {
        origin_odometry_ = *msg;
        moving_count_ += 1;
    }

    measuring_distance();

    if(origin_distance_ > 1)
    {
        moving_flag_ = true;
    }

    current_odometry_ = *msg;
}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    map_callback_flag_ = true;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
    laser_callback_flag_ = true;
}

//座標原点からの距離を計測する関数
void Localizer::measuring_distance()
{
    double origin_dx = current_odometry_.pose.pose.position.x - origin_odometry_.pose.pose.position.x;
    double origin_dy = current_odometry_.pose.pose.position.y - origin_odometry_.pose.pose.position.y;

    origin_distance_ = hypot(origin_dx, origin_dy);
}

//ノイズを付与する関数dist
double Localizer::set_noise(double mu, double dev)
{
    std::normal_distribution<> dist(mu, dev);
    return dist(engine);
}

//角度を-π~πに直す関数
double Localizer::optimize_angle(double angle)
{
    if(angle > M_PI){
        angle -= 2*M_PI;
   }
    if(angle < -M_PI){
        angle += 2*M_PI;
    }

    return angle;
}

//初期化処理を動かし方する関数
void Localizer::initialize()
{
    for(int i=0; i<particle_num_; i++)
    {
        double x = set_noise(init_x_, init_dev_);
        double y = set_noise(init_y_, init_dev_);
        double yaw = set_noise(init_yaw_, init_dev_);
        Particle p(x, y, yaw);
        particles_.push_back(p);
    }
}

//パーティクルの動作を更新する関数
void Localizer::motion_update()
{
    double dx = current_odometry_.pose.pose.position.x - previous_odometry_.pose.pose.position.x;
    double dy = current_odometry_.pose.pose.position.y - previous_odometry_.pose.pose.position.y;
    double cur_yaw = tf2::getYaw(current_odometry_.pose.pose.orientation);
    double pre_yaw = tf2::getYaw(previous_odometry_.pose.pose.orientation);
    double dyaw = optimize_angle(cur_yaw - pre_yaw);

    double distance = hypot(dx, dy);
    double direction = optimize_angle(atan2(dy, dx) - pre_yaw);

    for(auto& p : particles_){
        move(p, distance, direction, dyaw);
    }
}

//パーティクルを動かす関数
void Localizer::move(Particle& p, double distance, double direction, double rotation)
{
    distance += set_noise(0.0, distance * move_distance_dev_);
    direction += set_noise(0.0, direction * move_direction_dev_);
    rotation += set_noise(0.0, rotation * move_rotation_dev_);

    double x = p.get_pose_x() + distance * cos(optimize_angle(direction + p.get_pose_yaw()));
    double y = p.get_pose_y() + distance * sin(optimize_angle(direction + p.get_pose_yaw()));
    double yaw = optimize_angle(p.get_pose_yaw() + rotation);

    p.set_pose(x, y, yaw);
}

//パーティクルの観測を更新する関数
void Localizer::measurement_update()
{
    for(auto &p : particles_)
    {
        double new_weight = calculate_weight(p);
        p.set_weight(new_weight);
    }

    normalize_weight();

    //alpha_mean = 各パーティクルのlaser×1の重み（一様）
    double alpha_mean = alpha_ / (laser_.ranges.size() / laser_step_ * particle_num_);

    // ROS_INFO_STREAM("alpha_: "<<alpha_<<" alpha_mean: "<<alpha_mean);

    if(alpha_mean < alpha_th_ && expansion_count_ < expansion_limit_)
    {
        median_pose();
        expansion_count_ ++;
        expansion_reset();
        ROS_INFO("expansion reset");
    }
    else
    {
        weighted_mean_pose();
        expansion_count_ = 0;
        resampling();
        ROS_INFO("resampling");
    }
}

//weightを返す関数
double Localizer::calculate_weight(Particle& p)
{
    double weight = 0.0;
    double angle = optimize_angle(p.get_pose_yaw() + laser_.angle_min);
    double angle_step = laser_.angle_increment;
    int limit = laser_.ranges.size();

    for(int i=0; i < limit; i += laser_step_)
    {
        if(laser_.ranges[i] > laser_ignore_range_)
        {
            double map_distance = distance_on_map(p.get_pose_x(), p.get_pose_y(), angle);

            double sigma = laser_.ranges[i] * laser_distance_dev_;

            weight += likelihood(map_distance, laser_.ranges[i], sigma);
        }

        angle = optimize_angle(angle + angle_step * laser_step_);
    }
    // ROS_INFO("weight: %f", weight);
    return weight;
}

//パーティクルから壁までの距離を求める関数①
double Localizer::distance_on_map(double map_x, double map_y, double laser_angle)
{
    double search_step = map_.info.resolution;

    double search_limit = laser_.range_max / 2.0;

    for(double distance = 0.0; distance <= search_limit; distance += search_step)
    {
        map_x += search_step * cos(laser_angle);
        map_y += search_step * sin(laser_angle);

        int map_occupancy = get_map_occupancy(map_x, map_y);

        if(map_occupancy != 0){
            return distance;
        }
    }
    return search_limit;
}

//パーティクルから壁までの距離を求める関数②
int Localizer::get_map_occupancy(double x, double y)
{
    double origin_x = map_.info.origin.position.x;
    double origin_y = map_.info.origin.position.y;
    double resolution = map_.info.resolution;
    double width = map_.info.width;

    int mx = (int)floor( (x - origin_x) / resolution );
    int my = (int)floor( (y - origin_y) / resolution );

    int occupancy = map_.data[mx + my*width];

    return occupancy;
}

//確率密度関数を用いて重みを決める関数
double Localizer::likelihood(double x, double mu, double dev)
{
    double ans = exp( - pow(x - mu, 2) / (2.0 * pow(dev, 2)) ) / ( sqrt( 2.0 * M_PI ) * dev );

    return ans;
}

//重みの正規化を行う関数
void Localizer::normalize_weight()
{
    alpha_ = 0.0;
    for(const auto &p : particles_)
    {
        alpha_ += p.get_weight();
        // ROS_INFO("alpha_: %f", alpha_);
    }

    for(auto &p : particles_)
    {
        double new_weight = p.get_weight() / alpha_;
        p.set_weight(new_weight);
        // ROS_INFO("normalize_weight: %f", new_weight);
    }
}

//パーティクルの現在の位置を決める関数（中央値）
void Localizer::median_pose()
{
    std::vector<double> x_list;
    std::vector<double> y_list;
    std::vector<double> yaw_list;

    for(const auto& p : particles_)
    {
        x_list.push_back(p.get_pose_x());
        y_list.push_back(p.get_pose_y());
        yaw_list.push_back(p.get_pose_yaw());
    }

    const double x_median = get_median(x_list);
    const double y_median = get_median(y_list);
    const double yaw_median = get_median(yaw_list);
    estimated_pose_.set_pose(x_median, y_median, yaw_median);
}

//中央値を得る関数
double Localizer::get_median(std::vector<double>& data)
{
    sort(begin(data), end(data));
    if(data.size()%2 == 1)
    {
        return data[(data.size()-1) / 2];
    }
    else
    {
        return (data[data.size()/2 - 1] + data[data.size()/2]) / 2.0;
    }
}

//パーティクルの現在の位置を決める関数（加重平均）
void Localizer::weighted_mean_pose()
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double max_weight = get_max_weight();

    for(const auto &p : particles_)
    {
        x += p.get_pose_x() * p.get_weight();
        y += p.get_pose_y() * p.get_weight();
        if(p.get_weight() == max_weight)
            yaw = p.get_pose_yaw();
    }

    estimated_pose_.set_pose(x, y, yaw);
}

//particles_から最大のweightを得る関数
double Localizer::get_max_weight()
{
    double max = 0.0;

    for(const auto &p : particles_)
    {
        if(max <= p.get_weight()){
            max = p.get_weight();
        }
    }

    return max;
}

//膨張リセットを行う関数
void Localizer::expansion_reset()
{
    for(int i=0; i<particle_num_; i++)
    {
        double x = set_noise(particles_[i].get_pose_x(), expansion_reset_x_dev_);
        double y = set_noise(particles_[i].get_pose_y(), expansion_reset_y_dev_);
        double yaw = set_noise(particles_[i].get_pose_yaw(), expansion_reset_yaw_dev_);
        particles_[i].set_pose(x, y, yaw);
    }

    reset_weight();
}

//particles_の重みをリセットする関数
void Localizer::reset_weight()
{
    for(auto &p : particles_){
        p.set_weight(1.0/particles_.size());
    }
}

//MCLのリサンプリング（系統リサンプリング）をする関数
void Localizer::resampling()
{
    std::vector<double> ws;
    ws.push_back(particles_[0].get_weight());
    for(int i=1; i<particles_.size(); i++)
    {
        ws.push_back(ws.back() + particles_[i].get_weight());
    }

    const std::vector<Particle> old(particles_);
    const double step = ws.back() / particles_.size();
    const double start = (double)rand() / RAND_MAX * step;

    std::vector<int> chosen_indexes;
    int tick = 0;
    for(int i=0; i<particles_.size(); i++)
    {
        while(ws[tick] <= start + i*step)
        {
            tick++;
            if(tick == particles_.size())
            {
                ROS_ERROR("Resampling Failed");
                exit(1);
            }
        }
        chosen_indexes.push_back(tick);
    }

    for(int i=0; i<particles_.size(); i++)
    {
        particles_[i] = old[chosen_indexes[i]];
    }

    reset_weight();
}

//AMCLのリサンプリングをする関数
// void Localizer::AMCL_resampling()
// {
//     std::vector<Particle> new_particles;
//     new_particles.reserve(particle_num_);
//
//     std::uniform_int_distribution<> int_dist(0,particle_num_-1);
//     std::uniform_real_distribution<> double_dist(0.0,get_max_weight()*2.0);
//
//     alpha_slow_ += alpha_slow_th_ * (alpha_ - alpha_slow_);
//     alpha_fast_ += alpha_fast_th_ * (alpha_ - alpha_fast_);
//
//     num_replace_ = (int) (particle_num_ * std::max( 0.0, 1.0 - alpha_fast_ / alpha_slow_));
//
//     int index = int_dist(engine);
//     double beta = 0.0;
//
//     for(int i=0; i<particle_num_; i++)
//     {
//         if(new_particles.size() < num_replace_)
//         {
//             double x = set_noise(estimated_pose_.get_pose_x(), resampling_reset_x_dev_);
//             double y = set_noise(estimated_pose_.get_pose_y(), resampling_reset_y_dev_);
//             double yaw = set_noise(estimated_pose_.get_pose_yaw(), resampling_reset_yaw_dev_);
//             Particle p(x, y, yaw);
//             new_particles.push_back(p);
//         }
//
//         else
//         {
//             beta += double_dist(engine);
//
//             while(beta > particles_[index].get_weight())
//             {
//                 beta -= particles_[index].get_weight();
//                 index = (index+1) % particle_num_;
//             }
//
//             new_particles.push_back(particles_[index]);
//         }
//     }
//
//     particles_ = new_particles;
//     reset_weight();
// }


void Localizer::publish_particles()
{
    particle_cloud_msg_.header.stamp = ros::Time::now();
    particle_cloud_msg_.header.frame_id = "map";
    particle_cloud_msg_.poses.resize(particles_.size());

    for(int i=0; i<particles_.size(); i++)
    {
        particle_cloud_msg_.poses[i].position.x = particles_[i].get_pose_x();
        particle_cloud_msg_.poses[i].position.y = particles_[i].get_pose_y();
        particle_cloud_msg_.poses[i].position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, particles_[i].get_pose_yaw());
        tf2::convert(q, particle_cloud_msg_.poses[i].orientation);
    }

    pub_particle_cloud_.publish(particle_cloud_msg_);
}

void Localizer::publish_estimated_pose()
{
    estimated_pose_msg_.header.stamp = ros::Time::now();
    estimated_pose_msg_.header.frame_id = "map";

    estimated_pose_msg_.pose.position.x = estimated_pose_.get_pose_x();
    estimated_pose_msg_.pose.position.y = estimated_pose_.get_pose_y();

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, estimated_pose_.get_pose_yaw());
    tf2::convert(q, estimated_pose_msg_.pose.orientation);

    pub_estimated_pose_.publish(estimated_pose_msg_);
}

void Localizer::broadcast_roomba_state()
{
    double map_to_base_x = estimated_pose_.get_pose_x();
    double map_to_base_y = estimated_pose_.get_pose_y();
    double map_to_base_yaw = estimated_pose_.get_pose_yaw();

    double odom_to_base_x = current_odometry_.pose.pose.position.x;
    double odom_to_base_y = current_odometry_.pose.pose.position.y;
    double odom_to_base_yaw = tf2::getYaw(current_odometry_.pose.pose.orientation);

    double roomba_state_yaw = optimize_angle(map_to_base_yaw - odom_to_base_yaw);
    double roomba_state_x = map_to_base_x - odom_to_base_x * cos(roomba_state_yaw) + odom_to_base_y * sin(roomba_state_yaw);
    double roomba_state_y = map_to_base_y - odom_to_base_x * sin(roomba_state_yaw) - odom_to_base_y * cos(roomba_state_yaw);

    geometry_msgs::Quaternion roomba_state_q;
    tf::quaternionTFToMsg(tf::createQuaternionFromYaw(roomba_state_yaw), roomba_state_q);

    geometry_msgs::TransformStamped roomba_state;
    roomba_state.header.stamp = ros::Time::now();

    roomba_state.header.frame_id = "map";
    roomba_state.child_frame_id = "odom";

    roomba_state.transform.translation.x = roomba_state_x;
    roomba_state.transform.translation.y = roomba_state_y;
    roomba_state.transform.translation.z = 0.0;
    roomba_state.transform.rotation = roomba_state_q;

    roomba_state_broadcaster_.sendTransform(roomba_state);
    ROS_INFO("localizer_broadcast:%f", tf2::getYaw(roomba_state.transform.rotation));
}


void Localizer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(odometry_callback_flag_ && map_callback_flag_ && laser_callback_flag_)
        {
            if(init_request_flag_)
            {
                initialize();
                init_request_flag_ = false;

            }

            if(moving_flag_)
            {
                motion_update();
                measurement_update();
            }

            publish_particles();
            publish_estimated_pose();

            // try
            // {
            //     broadcast_roomba_state();
            // }
            // catch(tf::TransformException &ex)
            // {
            //     ROS_ERROR("%s", ex.what());
            // }
        }


        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}

