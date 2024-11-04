#pragma once
#include"ros/ros.h"
#include"Eigen/Core"
#include "random"

#include "searcher/search_algorithm.hpp"



class TopoSearch : public SearchAlgorithm {

public:
    explicit TopoSearch(std::shared_ptr<GlobalMap> global_map);
    std::vector<Eigen::Vector3d> getPath() override;
    bool search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) override;
    void init();
    
    std::vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt, double &dis_temp, int &exist_unvisiual);
    void checkHeightFeasible(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id);
    bool heightFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int &direction, double step = 0.05, double thresh = 0.10);
    Eigen::Vector3d getSample();
    void searchPaths(int node_id = 1);
    void DijkstraSearch(int node_id);    
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh, Eigen::Vector3d& pc, int caster_id);
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& colli_pt, double thresh);
    bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt);
    std::vector<Eigen::Vector3d> smoothTopoPath(std::vector<Eigen::Vector3d> topo_path);
    bool getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point);



    // 定义随机采样器
    std::random_device m_rd;
    std::default_random_engine m_eng;
    std::uniform_real_distribution<double> m_rand_pos;
    int max_sample_num;
    Eigen::Vector3d m_sample_inflate;  /// 采样膨胀点

};