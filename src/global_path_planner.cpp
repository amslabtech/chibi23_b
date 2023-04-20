#include "global_path_planner/global_path_planner.h"

Astar::Astar():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_checker_", map_checker_, {false});
    private_nh_.param("test_show_", test_show_, {true});
    private_nh_.param("sleep_time_", sleep_time_, {0.002});
//    private_nh_.param("path_checker", path_checker_, {false});
//    private_nh_.param("is_reached", is_reached_, {false});
//    private_nh_.param("wall_cost", wall_cost_, {1e10});

    global_path_.header.frame_id  = "map";
    current_node_.header.frame_id = "map";

    global_path_.poses.reserve(2000);

    sub_map_ = nh_.subscribe("/map", 1, &Astar::map_callback, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    if(test_show_)
    {
        pub_current_path_ = nh_.advertise<nav_msgs::Path>("/current_path", 1);
        pub_node_point_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_node", 1);
    }
}


void Astar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)  //マップの読み込み
{
    // ROS_INFO("map_callback was started...");
    map_ = *msg;
    origin_x_ = map_.info.origin.position.x;
    origin_y_ = map_.info.origin.position.y;
    height_ = map_.info.height;
    width_ = map_.info.width;
    resolution_ = map_.info.resolution;
    map_checker_ = true;
}

void Astar::get_way_points(std::vector<std::vector<int>>& list)  //経由点の宣言
{
    // ROS_INFO("getting waypoints started...");
    int x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5,x6,y6,x7,y7,x8,y8,x9,y9;
    x0 = x9 = - origin_x_ / resolution_;
    y0 = y9 = - origin_y_ / resolution_;
    x1 = round((8.88 - origin_x_) / resolution_);
    y1 = round((0.15 - origin_y_) / resolution_);
    x2 = round((9.52 - origin_x_) / resolution_);
    y2 = round((7.04 - origin_y_) / resolution_);
    x3 = round((9.18 - origin_x_) / resolution_);
    y3 = round((14.1 - origin_y_) / resolution_);
    x4 = round((-7.43 - origin_x_) / resolution_);
    y4 = round((14.25 - origin_y_) / resolution_);
    x5 = round((-24.39 - origin_x_) / resolution_);
    y5 = round((14.38 - origin_y_) / resolution_);
    x6 = round((-24.43 - origin_x_) / resolution_);
    y6 = round((7.20 - origin_y_) / resolution_);
    x7 = round((-24.58 - origin_x_) / resolution_);
    y7 = round((0.174 - origin_y_) / resolution_);
    x8 = round((-8.00 - origin_x_) / resolution_);
    y8 = round((0.166 - origin_y_) / resolution_);

    list = {
        {x0,y0},
        {x1,y1},
        {x2,y2},
        {x3,y3},
        {x4,y4},
        {x5,y5},
        {x6,y6},
        {x7,y7},
        {x8,y8},
        {x9,y9},
    };
}

Node Astar::set_way_point(const int phase)  //スタートとゴールの取得
{
    // ROS_INFO("setting waypoint started...");
    get_way_points(way_points_);
    Node way_point;
    way_point.x = way_points_[phase][0];
    way_point.y = way_points_[phase][1];
    return way_point;
}

double Astar::make_heuristic(const Node node)  //ヒューリスティック関数の計算
{
    // ROS_INFO("making heuristic started...");
    const double dx = (node.x - goal_node_.x);
    const double dy = (node.y - goal_node_.y);
    const double distance = hypot(dx,dy);

    return distance;
}

bool Astar::check_same_node(const Node n1, const Node n2)  //同じノードか確認
{
    // ROS_INFO("checking same node started...");
    if(n1.x == n2.x and n1.y == n2.y)
        return true;
    else
        return false;
}

int Astar::check_list(const Node target_node, std::vector<Node>& set)  //リストの中を検索
{
    for(int i=0;i<set.size();i++)
        if(check_same_node(target_node, set[i]))
            return i;
    return -1;
}

bool Astar::check_obs(const Node node)  //壁の確認
{
    // ROS_INFO("CHECK!!!");
    const int grid_index = node.x + (node.y * width_);
    // std::cout << "index = " << grid_index << std::endl;
    // std::cout << "grid_date =" << map_.data[grid_index] << std::endl;
    return map_.data[grid_index] == 1;
}

void Astar::swap_node(const Node node, std::vector<Node>& list1, std::vector<Node>& list2)  //リスト間のノードの移動
{
    // ROS_INFO("swaping node started...");
    const int list1_node_index = check_list(node,list1);
    if(list1_node_index == -1)
    {
        ROS_INFO("swap node was faird");
        exit(0);
    }

    list1.erase(list1.begin() + list1_node_index);
    list2.push_back(node);
    // std::cout << "swap (" << node.x << ", " << node.y << ")" << std::endl;
}

void Astar::show_node_point(const Node node)  //ノードの表示
{
    if(test_show_)
    {
        current_node_.point.x = node.x * resolution_ + origin_x_;
        current_node_.point.y = node.y * resolution_ + origin_y_;
        // current_node_.point.x = node.x;
        // current_node_.point.y = node.y;
        pub_node_point_.publish(current_node_);
        ros::Duration(sleep_time_).sleep();
    }
}

void Astar::show_path(nav_msgs::Path& current_path)  //パスの表示
{
    if(test_show_)
    {
        current_path.header.frame_id = "map";
        pub_current_path_.publish(current_path);
        ros::Duration(sleep_time_).sleep();
    }
}

Node Astar::select_min_f()  //f値が小さいものを取得
{
    // ROS_INFO("selecting min_f...");
    Node min_node = open_list_[0];
    double min_f = open_list_[0].f;

    for(const auto& open_node : open_list_)
    {
        if(open_node.f < min_f)
        {
            min_f = open_node.f;
            min_node = open_node;
        }
    }
    return min_node;
}

bool Astar::check_goal(const Node node)  //あんたゴール？
{
    return check_same_node(node,goal_node_);
}

bool Astar::check_start(const Node node)  //あんたスタート？
{
    return check_same_node(node,start_node_);
}

void Astar::update_list(const Node node)  //リストの更新
{
    // ROS_INFO("updating list started...");
    std::vector<Node> neighbors;
    int count = 1;

    create_neighbors(node, neighbors);

    for(const auto& neighbor : neighbors)
    {
        // std::cout << "neighbor " << count << " (" << neighbor.x << "," << neighbor.y << ")" << std::endl;

        if(check_obs(neighbor))
        {
            // ROS_INFO("there is obs");
            continue;
        }

        // std::cout << "check obs was comp..." << std::endl;

        int flag;
        int node_index;
        std::tie(flag, node_index) = search_node(neighbor);

        // std::cout << "flag = " << flag << ", node_index = " << node_index << ", count = " << count << std::endl;

        if(flag == -1)
        {
            open_list_.push_back(neighbor);
        }
        else if(flag == 1)
        {
            if(neighbor.f < open_list_[node_index].f)
            {
                open_list_[node_index].f = neighbor.f;
                open_list_[node_index].parent_x = neighbor.parent_x;
                open_list_[node_index].parent_y = neighbor.parent_y;
            }

        }
        else if(flag == 2)
        {
            if(neighbor.f < close_list_[node_index].f)
            {
                close_list_.erase(close_list_.begin() + node_index);
                open_list_.push_back(neighbor);
            }
        }
        count++;
    }
    // ROS_INFO("searching node ended...");
}

void Astar::create_neighbors(const Node node, std::vector<Node>&  neighbors)  //隣接ノードの作成
{
    // ROS_INFO("creating neighbors started...");
    std::vector<Motion> motion_list;
    get_motion(motion_list);
    const int motion_num = motion_list.size();

    for(int i=0;i<motion_num;i++)
    {
        Node neighbor = get_neighbor(node, motion_list[i]);
        // std::cout << "neighbor" << i + 1 << "= (" << neighbor.x << "," << neighbor.y << ")" << std::endl;
        neighbors.push_back(neighbor);
    }
}

void Astar::get_motion(std::vector<Motion>& list)  //動きの取得
{
    // ROS_INFO("getting motion started...");
    list.push_back(motion(1,0,1));
    list.push_back(motion(0,1,1));
    list.push_back(motion(-1,0,1));
    list.push_back(motion(0,-1,1));

    list.push_back(motion(1,1,sqrt(2)));
    list.push_back(motion(1,-1,sqrt(2)));
    list.push_back(motion(-1,1,sqrt(2)));
    list.push_back(motion(1,-1,sqrt(2)));
}

Motion Astar::motion(const int dx,const int dy,const int cost)  //もーしょん
{
    Motion motion;
    motion.dx = dx;
    motion.dy = dy;
    motion.cost = cost;

    return motion;
}

Node Astar::get_neighbor(const Node node, const Motion motion)  //隣接ノードの取得
{
    Node neighbor;

    neighbor.x = node.x + motion.dx;
    neighbor.y = node.y + motion.dy;

    neighbor.f = (node.f - make_heuristic(node)) + make_heuristic(neighbor) + motion.cost;

    neighbor.parent_x = node.x;
    neighbor.parent_y = node.y;

    return neighbor;
}

std::tuple<int, int> Astar::search_node(const Node node)  //どこのリストに入っているかの検索
{
    // ROS_INFO("searching node started...");
    const int open_list_index = search_node_from_list(node,open_list_);
    if(open_list_index != -1)
    {
        // ROS_INFO("found in open");
        return std::make_tuple(1,open_list_index);
    }

    const int close_list_index = search_node_from_list(node,close_list_);
    if(close_list_index != -1)
    {
        // ROS_INFO("found in close");
        return std::make_tuple(2,close_list_index);
    }
    // ROS_INFO("there is no node in list");
    return std::make_tuple(-1,-1);
}

void Astar::create_path(Node node)  //パスの作成
{
    ROS_INFO("making path...");
    nav_msgs::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));
    // ROS_INFO("qawsedrftghujikol");u
    int count = 0;
    // std::cout << "size = " << close_list_.size() << std::endl;

    while(not check_start(node))
    {
        // std::cout << "count = " << count << std::endl;
        for(int i=0;i<close_list_.size();i++)
        {
            // std::cout << "i = " << i << std::endl;
            // std::cout << "(" << close_list_[i].x << ", " << close_list_[i].y << ")" << std::endl;
            if(check_parent(i,node))
            {
                node = close_list_[i];
                // show_node_point(node);
                partial_path.poses.push_back(node_to_pose(node));
                // ROS_INFO("create path was ended...");
                break;
            }
            if(i == close_list_.size()-1)
            {
                // std::cout << "(" << close_list_[i].x << ", " << close_list_[i].y << ")" << std::endl;
                ROS_INFO("sippai.......");
                exit(0);
            }
        }
        // count++;
    }
    reverse(partial_path.poses.begin(),partial_path.poses.end());
    show_path(partial_path);
    global_path_.poses.insert(global_path_.poses.end(),partial_path.poses.begin(),partial_path.poses.end());
}

bool Astar::check_parent(const int index, const Node node)  //親ノードの確認
{
    bool check_x = close_list_[index].x == node.parent_x;
    bool check_y = close_list_[index].y == node.parent_y;

    return check_x and check_y;
}

geometry_msgs::PoseStamped Astar::node_to_pose(const Node node)  //ノードから座標の検索
{
    // ROS_INFO("Node To Pose was started...");
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = node.x * resolution_ + origin_x_;
    pose_stamped.pose.position.y = node.y * resolution_ + origin_y_;
    // pose_stamped.pose.position.x = node.x * resolution_ + map_.info.origin.position.x;
    // pose_stamped.pose.position.y = node.y * resolution_ + map_.info.origin.position.y;
    return pose_stamped;
}

void Astar::show_exe_time()
{
     ROS_INFO_STREAM("Duration = " << std::fixed << std::setprecision(2) << ros::Time::now().toSec() - begin_.toSec() << "s");
}

int Astar::search_node_from_list(const Node node, std::vector<Node>& list)  //リストの中を検索
{
    // ROS_INFO("searching node from list started...");
    for(int i=0;i<list.size();i++)
        if(check_same_node(node, list[i]))
        {
            // ROS_INFO("searching node from list success...");
            return i;
        }
    // ROS_INFO("searching node from list ended...");
    return -1;
}

void Astar::planning()  //経路計画
{
    begin_ = ros::Time::now();
    const int total_phase = 9;
    for(int phase=0;phase<total_phase;phase++)
    {
        printf("phase %d is starting...\n",phase);
        open_list_.clear();
        close_list_.clear();

        start_node_ = set_way_point(phase);
        goal_node_ = set_way_point(phase + 1);
        start_node_.f = make_heuristic(start_node_);
        open_list_.push_back(start_node_);
        std::cout << "startnode (" << start_node_.x << "," << start_node_.y << "," << start_node_.f << ")" << std::endl;
        std::cout << "goalnode (" << goal_node_.x << "," << goal_node_.y << "," << goal_node_.f << ")" << std::endl;

        while(ros::ok())
        {
            Node min_node = select_min_f();
            // std::cout << "current_node =(" << min_node.x << "," << min_node.y << ")" << std::endl;
            show_node_point(min_node);

            if(check_goal(min_node))
            {
                create_path(min_node);
                // ROS_INFO("hogehoGe");
                break;
            }
            else
            {
                // ROS_INFO("replace in close...");
                swap_node(min_node,open_list_,close_list_);
                // std::cout << "( " << min_node.x << ", " << min_node.y << ")" << std::endl;
                // std::cout << "size of close = " << close_list_.size() << std::endl;
                update_list(min_node);
                // ROS_INFO("wassyoi!");
            }
        }
        // ROS_INFO("hugahuga");
    }
    pub_path_.publish(global_path_);
    show_exe_time();
    ROS_INFO("COMPLITE ASTAR PROGLAM");
    exit(0);
}

void Astar::process()  //メイン関数で実行する関数
{
    ROS_INFO("process is starting...");
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(!map_checker_)
            ROS_INFO("NOW LOADING...");
        else
            planning();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planner");
    Astar astar;
    astar.process();
    return 0;
}
