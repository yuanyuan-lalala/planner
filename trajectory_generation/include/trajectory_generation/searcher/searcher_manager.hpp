#pragma once
#include "searcher/search_algorithm.hpp"
#include "searcher/Astar_searcher.hpp"
#include "searcher/topo_searcher.hpp"
#include <memory>
#include <iostream>

class SearcherManager {
public:
    enum class AlgorithmType { ASTAR, TOPO };

    explicit SearcherManager(AlgorithmType type,std::shared_ptr<GlobalMap> global_map);
    

    std::vector<Eigen::Vector3d> smoothTopoPath(const std::vector<Eigen::Vector3d>& origin_path);
    
    void setPoints(Eigen::Vector3d start_pt,Eigen::Vector3d end_pt);
    
    void executeSearch();

    std::unique_ptr<SearchAlgorithm> algorithm;
    std::shared_ptr<GlobalMap> m_global_map;

private:
    
    
    Eigen::Vector3d m_start_pt;
    Eigen::Vector3d m_end_pt;


   
};
