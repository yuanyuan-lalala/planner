#include"searcher/search_algorithm.hpp"

SearchAlgorithm::SearchAlgorithm(std::shared_ptr<GlobalMap> global_map):m_globalMap(global_map){}





double SearchAlgorithm::getHeu(std::shared_ptr<GridNode> node1, std::shared_ptr<GridNode> node2)
{
/** 选择计算方式 */
#define USING_MANHATTAN
    /* 获取两点的三维距离 */
    int dx = abs(node2->m_index[0] - node1->m_index[0]);
    int dy = abs(node2->m_index[1] - node1->m_index[1]);
    int dz = abs(node2->m_index[2] - node1->m_index[2]);

/* 使用Manhattan距离来得到h */
#ifdef USING_MANHATTAN
    return pow((dx) * (dx) + (dy) * (dy) + (dz) * (dz), 0.5);
#endif

/* 使用Euclidean距离来得到h */
#ifdef USING_EUCLIDEAN
    return dx + dy + dz;
#endif

/* 使用USING_DIJKSTRA（h = 0） */
#ifdef USING_DIJKSTRA
    return 0;
#endif

    return 0;
}


bool SearchAlgorithm::findNeighPoint(Eigen::Vector3i path_point, Eigen::Vector3i &output_point, int check_num)
{
    Eigen::Vector3i path_succ;
    std::vector<double> path_distance;
    std::vector<Eigen::Vector3i> path_free;
    for (int idx = -check_num; idx <= check_num; idx++)
    {
        for (int jdx = -check_num; jdx <= check_num; jdx++)
        {
            path_succ.x() = path_point.x() + idx;
            path_succ.y() = path_point.y() + jdx;
            path_succ.z() = path_point.z();
            if (m_globalMap->isFree(path_succ))
            {
                path_free.push_back(path_succ);
                double distance = sqrt(pow(path_succ.x() - path_point.x(), 2) + pow(path_succ.y() - path_point.y(), 2));
                path_distance.push_back(distance);
                // output_point = path_succ;
                // return true;
            }
        }
    }
    if (path_free.empty())
    {
        return false;
    }
    std::vector<double>::iterator maxdistance = std::max_element(path_distance.begin(), path_distance.end());
    int index = std::distance(path_distance.begin(), maxdistance);
    output_point = path_free[index];
    return true;
}

