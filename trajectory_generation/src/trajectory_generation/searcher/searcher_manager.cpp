#include "searcher/searcher_manager.hpp"

SearcherManager::SearcherManager(AlgorithmType type,std::shared_ptr<GlobalMap> global_map) {
        m_global_map = global_map;
        switch (type) {
            case AlgorithmType::ASTAR:
                algorithm = std::make_unique<AstarSearch>(m_global_map);
                break;
            case AlgorithmType::TOPO:
                algorithm = std::make_unique<TopoSearch>(m_global_map);
                break;
            default:
                std::cerr << "Unknown algorithm type." << std::endl;
                break;
        }
}

void SearcherManager::setPoints(Eigen::Vector3d start_pt,Eigen::Vector3d end_pt){
    m_start_pt = start_pt;
    m_end_pt = end_pt;
}

void SearcherManager::executeSearch() {
        if (algorithm) {
            algorithm->search(m_start_pt, m_end_pt);
        } else {
            std::cerr << "Algorithm not set." << std::endl;
        }
}


// 新增的 smoothTopoPath 实现
std::vector<Eigen::Vector3d> SearcherManager::smoothTopoPath(const std::vector<Eigen::Vector3d>& origin_path) {
    // 使用 dynamic_cast 将 algorithm 转换为 TopoSearch 类型
    if (auto topo_search = dynamic_cast<TopoSearch*>(algorithm.get())) {
        return topo_search->smoothTopoPath(origin_path); // 调用 TopoSearch 的特有方法
    } else {
        std::cerr << "Algorithm is not TopoSearch, cannot smooth topo path." << std::endl;
        return {}; // 返回空路径，或者根据需要处理错误情况
    }
}