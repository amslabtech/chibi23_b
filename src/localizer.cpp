#include "localizer/localizer.h"

std::random_device seed_gen;
std::mt19937 engine(seed_gen());

Particle::Particle(double x, double y, double yaw, double weight)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    weight_ = weight;
}

Localizer::Localizer():private_nh_("~")
{
    private_nh_.Param("hz", hz_, 10) ;
    private_nh_.Param("particle_num", particle_num_, 100) ;

    sub_laser_ = nh_.subscribe("/scan", 10, &Localizer::laser_callback, this);
    sub_odometry_ = nh_.subscribe("/roomba/odometry", 10, &Localizer::odometry_callback, this);
    sub_map_ = nh_.subscribe("/map", 10, &Localizer::map_callback, this);

    pub_estimated_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 1);
    pub_particle_cloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud", 1);
}

void Localizer::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{



}

void Localizer::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_ = *msg;
    map_callback_flag_ = true;
}

void Localizer::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}


//ノイズを付与する関数
double Localizer::set_noise(double mu, double dev)
{
    std::normal_distribution<> dist(mu, dev);
    return dist(enging frame_ie);
}


//初期化処理の関数
void Localizer::initialize()
{
    for(int i=0; i<num; i++)
    {
        double x = set_noise(init_x_, init_dev_);
        double y = set_noise(init_y_, init_dev_);
        double yaw = set_noise(init_yaw_, init_dev_);
        Particle p(x, y, yaw);
        particles_.push_back(p);
    }
}

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

void Localizer::process()
{
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(map_callback_flag_)
        {
            initialize();
            publish_particles();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}


int main(int argc. char **argv)
{
    ros::init(argc, argv, "localizer");
    Localizer localizer;
    localizer.process();

    return 0;
}









