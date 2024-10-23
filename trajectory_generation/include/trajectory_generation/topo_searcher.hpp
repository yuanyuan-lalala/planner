#pragma once
#include"ros/ros.h"
#include"global_map.hpp"
#include "random"

class GraphNode
{
public:
    enum NODE_TYPE{
        Guard = 1,
        Connector = 2
    };
    enum NODE_STATE{
        NEW = 1,
        CLOSE = 2,
        OPEN = 3
    };

    GraphNode() {}
    GraphNode(Eigen::Vector3d position, NODE_TYPE type, int id) {
        pos = position;
        type_ = type;
        state_ = NEW;
        m_id = id;
    }
    ~GraphNode() {}

    std::vector<std::shared_ptr<GraphNode>> neighbors;
    std::vector<std::shared_ptr<GraphNode>> neighbors_but_noconnected;
    NODE_TYPE type_;
    NODE_STATE state_;

    Eigen::Vector3d pos;
    int m_id;
    typedef std::shared_ptr<GraphNode> Ptr;
};


class TopoSearcher{
public:
    void init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap> &global_map);
    void createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
    std::vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt, double &dis_temp, int &exist_unvisiual);
    void checkHeightFeasible(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id);
    bool heightFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int &direction, double step = 0.05, double thresh = 0.10);
    Eigen::Vector3d getSample();
    std::vector<std::vector<Eigen::Vector3d>> searchPaths(int node_id = 1);
    void DijkstraSearch(int node_id);    
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh, Eigen::Vector3d& pc, int caster_id);
    bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);

    std::shared_ptr<GlobalMap> global_map_;
    std::vector<GraphNode::Ptr> m_graph;  /// 得到采样图

    std::vector<std::vector<Eigen::Vector3d>> raw_paths;
    std::vector<std::vector<Eigen::Vector3d>> final_paths;  // k 最优
    std::vector<Eigen::Vector3d> min_path;

    
    
    // 定义随机采样器
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;
    int max_sample_num;
    Eigen::Vector3d m_sample_inflate;  /// 采样膨胀点

};