#include "global_path_planner/global_path_planner.h"

Astar::Astar():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    private_nh_.param("map_checker", map_checker_, {false});
    private_nh_.param("test_show_", map_checker_, {true});
//    private_nh_.param("path_checker", path_checker_, {false});
//    private_nh_.param("is_reached", is_reached_, {false});
//    private_nh_.param("wall_cost", wall_cost_, {1e10});

    global_path_.header.frame_id  = "map";
    current_node_.header.frame_id = "map";
    sub_map_ = nh_.subscribe("/map", 10, &Astar::map_callback, this);
    pub_path_ = nh_.advertise<nav_msgs::Path>("/global_path", 1);
    if(test_show_)
    {
        pub_current_path_ = nh_.advertise<nav_msgs::Path>("/current_path", 1);
        pub_node_point_ = nh_.advertise<nav_msgs::Path>("/current_node", 1);
    }
}


void Astar::map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)  //マップの読み込み
{
    map_ = *msg;
    height_ = map_.info.height;
    width_ = map_.info.width;
    resolution_ = map_.info.resolution;

    map_checker_ = true;
    ROS_INFO("wassyoi");
}

void Astar::get_way_points(std::vector<std::vector<double>>& list)  //経由点の宣言
{
    double x0,y0,x1,y1,x2,y2,x3,y3,x4,y4,x5,y5;
    x0 = x5 = y0 = y5 = 0.0;
    x1 = 13.0;
    y1 = -0.50;
    x2 = 14.5;
    y2 = 13.4;
    x3 = -20.0;
    y3 = 15.0;
    x4 = -20.0;
    y4 = 1.00;

    list = {
        {x0,y0},
        {x1,y1},
        {x2,y2},
        {x3,y3},
        {x4,y4},
        {x5,y5},
    };
}

Node Astar::set_way_point(const int phase)  //スタートとゴールの取得
{
    get_way_points(way_points_);
    Node way_point;
    way_point.x = way_points_[phase][0];
    way_point.y = way_points_[phase][1];
    return way_point;
}

double Astar::make_heuristic(const Node node)  //ヒューリスティック関数の計算
{
    const double dx = (node.x - goal_node_.x);
    const double dy = (node.y - goal_node_.y);
    const double distance = hypot(dx,dy);

    return distance;
}

bool Astar::check_same_node(const Node n1, const Node n2)  //同じノードか確認
{
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
    const int grid_index = node.x + (node.y * width_);
    return map_.data[grid_index] == 100;
}

void Astar::swap_node(const Node node, std::vector<Node>& list1, std::vector<Node>& list2)  //リスト間のノードの移動
{
    const int list1_node_index = check_list(node,list1);
    if(list1_node_index == -1)
    {
        ROS_INFO("orannde");
        exit(0);
    }

    list1.erase(list1.begin() + list1_node_index);
    list2.push_back(node);
}

void Astar::show_node_point(const Node node)  //ノードの表示
{
    if(test_show_)
    {
        current_node_.point.x = node.x * resolution_ + map_.info.origin.position.x;
        current_node_.point.y = node.y * resolution_ + map_.info.origin.position.y;
        pub_node_point_.publish(current_node_);
    }
}

void Astar::show_path(nav_msgs::Path& current_path)  //パスの表示
{
    if(test_show_)
    {
        current_path.header.frame_id = "map";
        pub_current_path_.publish(current_path);
    }
}

Node Astar::select_min_f()  //f値が小さいものを取得
{
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
    std::vector<Node> neighbors;

    create_neighbors(node, neighbors);

    for(const auto& neighbor : neighbors)
    {
        if(check_obs(neighbor))
            continue;

        int flag;
        int node_index;
        std::tie(flag, node_index) = search_node(neighbor);

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
            if(neighbor.f < open_list_[node_index].f)
            {
                close_list_.erase(close_list_.begin() + node_index);
                open_list_.push_back(neighbor);
            }
        }
    }
}

void Astar::create_neighbors(const Node node, std::vector<Node>&  neighbors)  //隣接ノードの作成
{
    std::vector<Motion> motion_list;
    get_motion(motion_list);
    const int motion_num = motion_list.size();

    for(int i=0;i<motion_num;i++)
    {
        Node neighbor = get_neighbor(node, motion_list[i]);
        neighbors.push_back(neighbor);
    }
}

void Astar::get_motion(std::vector<Motion>& list)  //動きの取得
{
    list.push_back(motion(1,0,1));
    list.push_back(motion(0,1,1));
    list.push_back(motion(-1,0,1));
    list.push_back(motion(0,-1,1));
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

    neighbor.f = (node.f - make_heuristic(node) + make_heuristic(neighbor) + motion.cost);

    neighbor.parent_x = node.x;
    neighbor.parent_y = node.y;

    return neighbor;
}

std::tuple<int, int> Astar::search_node(const Node node)  //どこのリストに入っているかの検索
{
    const int open_list_index = search_node_from_list(node,open_list_);
    if(open_list_index != 1)
        return std::make_tuple(1,open_list_index);

    const int close_list_index = search_node_from_list(node,close_list_);
    if(close_list_index != 1)
        return std::make_tuple(2,close_list_index);

        return std::make_tuple(-1,-1);
}

void Astar::create_path(Node node)  //パスの作成
{
    nav_msgs::Path partial_path;
    partial_path.poses.push_back(node_to_pose(node));

    while(not check_start(node))
    {
        for(int i=0;i<close_list_.size();i++)
        {
            if(check_parent(i,node))
            {
                node = close_list_[i];
                show_node_point(node);
                partial_path.poses.push_back(node_to_pose(node));
                break;
            }
        }
    }
}

bool Astar::check_parent(const int index, const Node node)  //親ノードの確認
{
    bool check_x = close_list_[index].x == node.parent_x;
    bool check_y = close_list_[index].y == node.parent_y;

    return check_x and check_y;
}

geometry_msgs::PoseStamped Astar::node_to_pose(const Node node)  //ノードから座標の検索
{
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = node.x * map_.info.resolution + map_.info.origin.position.x;
    pose_stamped.pose.position.y = node.y * map_.info.resolution + map_.info.origin.position.y;
    return pose_stamped;
}


int Astar::search_node_from_list(const Node node, std::vector<Node>& list)  //リストの中を検索
{
    for(int i=0;i<list.size();i++)
        if(check_same_node(node, list[i]))
            return i;
    return -1;
}

void Astar::planning()  //経路計画
{
    const int total_phase = 5;
    for(int phase=0;phase<total_phase;phase++)
    {
        open_list_.clear();
        close_list_.clear();

        start_node_ = set_way_point(phase);
        goal_node_ = set_way_point(phase + 1);
        start_node_.f = make_heuristic(start_node_);
        open_list_.push_back(start_node_);

        ROS_INFO("ittyauyon");

        while(ros::ok())
        {
            Node min_node = select_min_f();
            show_node_point(min_node);

            if(check_goal(min_node))
            {
                create_path(min_node);
                break;
            }
            else
            {
                swap_node(min_node,open_list_,close_list_);
                update_list(min_node);
            }
        }
    }
}

void Astar::process()  //メイン関数で実行する関数
{
    ROS_INFO("hajimattayo");
    ros::Rate loop_rate(hz_);

    while(ros::ok())
    {
        if(map_checker_)
        {
            planning();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "global_path_planner");
    Astar astar;
    astar.process();
    return 0;
}
