#pragma once
#include"Eigen/Core"
#include"vector"
#include"memory"
// #include "unordered_map"
#include <unordered_set>
#include"map/global_map.hpp"
#include "searcher/graph_node.hpp"

class SearchAlgorithm {
public:

    explicit SearchAlgorithm(std::shared_ptr<GlobalMap> global_map);
    virtual ~SearchAlgorithm() = default;
    virtual bool search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) = 0;
    virtual std::vector<Eigen::Vector3d> getPath() = 0;
	std::multimap<double, GridNodePtr> openSet;
	std::multimap<double, GridNodePtr> closeSet;
    
    
    
    
    double getHeu(std::shared_ptr<GridNode> node1, std::shared_ptr<GridNode> node2);
	void AstarGetSucc(std::shared_ptr<GridNode> currentPtr, std::vector<std::shared_ptr<GridNode>> &neighborPtrSets, std::vector<double> &edgeCostSets);

    bool checkPathCollision(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d &collision_pos, Eigen::Vector3d cur_pos,
							Eigen::Vector3d &collision_start_point, Eigen::Vector3d &collision_target_point,
                            int &path_start_id, int &path_end_id); /// 检查优化完成后路径是否被动态障碍物遮挡
    bool checkPathCollision(std::vector <Eigen::Vector3d> optimized_path, Eigen::Vector3d &collision_pos);
	void getCurPositionIndex(std::vector<Eigen::Vector3d> optimized_path, Eigen::Vector3d cur_pos, int &cur_start_id);
    bool checkPointCollision(Eigen::Vector3i path_point, int check_swell);							  /// 检查路径是否可行
	bool findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num);	  /// 找附近最近的非障碍物点
	bool getNearPoint(Eigen::Vector3d headPos, Eigen::Vector3d tailPos, Eigen::Vector3d &best_point);
    void optimialPathUpdate(std::vector <Eigen::Vector3d> &optimized_path, Eigen::Vector3d start_point,
                            Eigen::Vector3d target_point);
    
    std::shared_ptr<GlobalMap> m_globalMap;
    std::vector<std::vector<Eigen::Vector3d>> m_raw_paths;
    std::vector<std::vector<Eigen::Vector3d>> m_final_paths;  // k 最优
    std::vector<Eigen::Vector3d> m_min_path;

    std::vector<GraphNode::Ptr> m_graph;//采样图
    std::vector<GraphNode::Ptr> m_graph_vis;  /// 用于可视化
    private:
    



};