#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>

struct Node
{
    int    x = 0;
    int    y = 0;
    int    parent_x = -1;
    int    parent_y = -1;
    double f = 0.0;
};

struct Motion
{
    int dx;
    int dy;
    double cost;
};

class Astar
{
public:
    Astar();
    void process();

private:
    void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& mag);  //マップの読み込み
    void swap_node(const Node node, std::vector<Node>& list1,std::vector<Node>& list2);  //リスト間の移動
    void show_node_point(const Node node);  //ノードの表示
    void show_path(nav_msgs::Path& current_path);  //パスの表示
    void get_way_points(std::vector<std::vector<double>>& list);  //経由点の宣言
    void planning();  //経路計画
    void create_path(Node node);  //経路作成
    void update_list(const Node node);  //隣接ノードの更新
    void create_neighbors(const Node node, std::vector<Node>& nodes);  //隣接ノードの作成
    void get_motion(std::vector<Motion>& motion);  //動作の定義

    bool check_same_node(const Node n1, const Node n2);  //同じノードかどうかの確認
    bool check_obs(const Node node);  //壁か判断
    bool check_start(const Node node);  //スタートかどうか
    bool check_goal(const Node node);  //ゴールかどうか
    bool check_parent(const int index, const Node node);

    double make_heuristic(const Node node);  //ヒューリスティック関数の取得
    int check_list(const Node target_node, std::vector<Node>& set);  //リストの中を検索
    int search_node_from_list(const Node node, std::vector<Node>& list);  //リストの中を検索
    Node set_way_point(const int phase);  //経由点の取得
    Node select_min_f();  //一番f値が小さいものを選択
    Node get_neighbor(const Node node, const Motion motion);  //隣接ノードの取得
    Motion motion(const int dx,const int dy,const int cost);  //もーしょん
    geometry_msgs::PoseStamped node_to_pose(const Node node);  //ノードから座標に変換

    std::tuple<int, int> search_node(const Node node);  //どっちのリストに入っているのか検索

    std::vector<double> origin_x_;  //原点のx座標
    std::vector<double> origin_y_;  //原点のy座標
    std::vector<std::vector<double>> way_points_;  //経由点たち

    int hz_;  //周波数
    Node start_node_;  //開始位置
    Node goal_node_;  //目標位置
    std::vector<Node> open_list_;  //opneリスト
    std::vector<Node> close_list_;  //closeリスト

    bool   map_checker_;  //正しくマップはとれたかな
    bool   test_show_;  //デバッグしますか

    //map製作用
    int height_;  //マップの幅だか高さだか
    int width_;  //マップの幅だか高さだか
    double resolution_;  //マップの解像度
    Node origin_;  //マップの原点
    std::vector<std::vector<int>> map_grid_;  //グリッドマップ

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_map_;
    ros::Publisher pub_path_;
    ros::Publisher pub_node_point_;
    ros::Publisher pub_current_path_;

    nav_msgs::Path global_path_;
    nav_msgs::OccupancyGrid map_;
    geometry_msgs::PointStamped current_node_;
};
#endif
