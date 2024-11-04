#pragma once
#include "searcher/search_algorithm.hpp"
#include <iostream>
#include "vector"

class AstarSearch : public SearchAlgorithm {
public:
    explicit AstarSearch(std::shared_ptr<GlobalMap> global_map);

    std::vector<Eigen::Vector3d> getPath() override ;
    bool search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt) override;
    void AstarGetSucc(std::shared_ptr<GridNode> currentPtr, std::vector<std::shared_ptr<GridNode>> &neighborPtrSets, std::vector<double> &edgeCostSets);
    void visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag);
    void visGridMap();
    double max_slope_height_diff = 0.1;
    

};