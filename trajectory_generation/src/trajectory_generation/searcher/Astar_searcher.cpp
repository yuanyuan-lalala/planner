#include"searcher/Astar_searcher.hpp"

AstarSearch::AstarSearch(std::shared_ptr<GlobalMap> global_map):SearchAlgorithm(global_map){

}

std::vector<Eigen::Vector3d> AstarSearch::getPath(){
    return m_min_path;
}


bool AstarSearch::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt){

/* 获取当前时间 */
    clock_t time_1 = clock();

    /* 将点云坐标映射为栅格ID(栅格ID从地图左下角算起) */
    Eigen::Vector3i start_index = m_globalMap->coord2gridIndex(start_pt);
    Eigen::Vector3i end_index = m_globalMap->coord2gridIndex(end_pt);



    /* 将栅格ID映射到点云坐标（点云坐标的原点在地图的地面中心） */
    start_pt = m_globalMap->gridIndex2coord(start_index);
    end_pt = m_globalMap->gridIndex2coord(end_index);

    /* 定义起始节点指针和目标节点指针 */
    std::shared_ptr<GridNode> startPtr = std::make_shared<GridNode>(start_index, start_pt);
    std::shared_ptr<GridNode> endPtr = std::make_shared<GridNode>(end_index, end_pt);
    

    /* 清空open set*/
    openSet.clear();
    m_min_path.clear();
    /* 定义当前节点指针和当前节点的邻居节点指针 */
    std::shared_ptr<GridNode> currentPtr = nullptr;
    std::shared_ptr<GridNode> neighborPtr = nullptr;

    /* 把起点放入open set */
    startPtr->m_gScore = 0;
    startPtr->m_fScore = getHeu(startPtr, endPtr);//首尾距离
    startPtr->m_set_type   = GridNode::setType::UNEXPLORE;
    startPtr->m_coord = start_pt;
    
    ROS_ERROR("Astar3 [START POINT]: [%f, %f, %f], [END POINT]: [%f, %f, %f]", 
         start_pt.x(), start_pt.y(), start_pt.z(), 
         end_pt.x(), end_pt.y(), end_pt.z());

    openSet.emplace(startPtr->m_fScore, startPtr);

    /* 定义邻居节点容器 */
    std::vector<GridNodePtr> neighborPtrSets;
    /* 定义边际开销容器*/
    std::vector<double> edgeCostSets;

    /* 循环排空open set */
    while (!openSet.empty())
    {
        /* 弹出open set的队首节点(当前处于openlist中f值最小的节点) */
        currentPtr = openSet.begin()->second;
        openSet.erase(openSet.begin());

        /* 修改这个节点的状态为已扩展 */
        currentPtr->m_set_type = GridNode::setType::OPENSET;

        ROS_ERROR("[INDEX] :%d,%d,%d",currentPtr->m_index.x(),currentPtr->m_index.y(),currentPtr->m_index.z());
        
        /* 如果当前节点就是终止节点则直接退出 */
        if (currentPtr->m_index == end_index)
        {
            clock_t time_2 = clock();
            std::shared_ptr<GridNode> tempPtr = currentPtr;
            while (tempPtr->m_index != start_index) {
                m_min_path.push_back(tempPtr->m_coord);
                tempPtr = tempPtr->m_cameFrom;
            }
            std::reverse(m_min_path.begin(), m_min_path.end());
            ROS_ERROR("[MIN_PATH_SIZE]:%zu",m_min_path.size());
            ROS_ERROR("[MIN_PATH]: [%f,%f,%f]",m_min_path[0].x(),m_min_path[0].y(),m_min_path[0].z());
            return true;
        }

        /* 按照一定次序找到所有邻居节点并计算他们的f值 */
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

        /* 遍历每个邻居节点 */
        for (int i = 0; i < (int)neighborPtrSets.size(); i++)
        {
            neighborPtr = neighborPtrSets[i];
            double tentative_gScore = currentPtr->m_gScore + edgeCostSets[i];
            /** 根据邻居节点的状态来对其进行相应的处理 **/
            /* id == 0 ：新节点（不在open set也不在close set中） */
            if (neighborPtr->m_set_type == GridNode::setType::UNEXPLORE) 
            {
                /* 标记该邻居节点的父节点为当前节点并将其放入open set */
                neighborPtr->m_cameFrom = currentPtr;
                neighborPtr->m_set_type = GridNode::setType::OPENSET;
                neighborPtr->m_gScore = tentative_gScore;
                neighborPtr->m_fScore = neighborPtr->m_gScore + getHeu(neighborPtr, endPtr);
                openSet.insert(std::make_pair(neighborPtr->m_fScore, neighborPtr));
                continue;
            }
            /* id == 1 ：已处于open set的节点（暂未拓展邻居的节点）*/
            else if (neighborPtr->m_set_type == GridNode::setType::OPENSET) {
                if (tentative_gScore < neighborPtr->m_gScore) {
                    neighborPtr->m_gScore = tentative_gScore;
                    neighborPtr->m_fScore = neighborPtr->m_gScore + getHeu(neighborPtr, endPtr);
                    neighborPtr->m_cameFrom = currentPtr;
                    // 插入更新后的节点（通过重新插入来更新优先级）
                    openSet.insert(std::make_pair(neighborPtr->m_fScore, neighborPtr));
                }
                continue;
            }
        }
    }
    clock_t time_2 = clock();
    double elapsed_time = static_cast<double>(time_2 - time_1) / CLOCKS_PER_SEC;
    if (elapsed_time > 0.1)
        ROS_ERROR("[AstarPathFinder] A* Search failed: No path found.");
    return false;


}

void AstarSearch::AstarGetSucc(std::shared_ptr<GridNode> currentPtr, std::vector<std::shared_ptr<GridNode>> &neighborPtrSets, std::vector<double> &edgeCostSets)
{
    /* 清空容器 */
    neighborPtrSets.clear();
    edgeCostSets.clear();

    /* 查找8领域 */
    for (int i = -1; i < 2; i++)
    {
        for (int j = -1; j < 2; j++)
        {
            /* 获取邻居节点的栅格ID */
            int _X = currentPtr->m_index[0] + i;
            int _Y = currentPtr->m_index[1] + j;
            int _Z = currentPtr->m_index[2];
            /* 将有效节点（避免重复访问、越界和碰上障碍物）加入邻居节点容器 */
            if (m_globalMap->isFree(_X, _Y, _Z))
            {

                /* 避免重复访问 */
                if ((*m_globalMap->GridNodeMap)[_X][_Y]->m_set_type != GridNode::setType::UNEXPLORE)
                {
                    Eigen::Vector3i center_index = currentPtr->m_index;
                    double center_height =  m_globalMap->getHeight(center_index[0], center_index[1]);
                    //旁边的节点
                    double height = m_globalMap->getHeight(_X,_Y);
                    if(height-center_height>max_slope_height_diff){
                        continue;
                    }
                    // m_globalMap->GridNodeMap
                    Eigen::Vector3d temp_coord =(*m_globalMap->GridNodeMap)[_X][_Y]->m_coord;
                    temp_coord[2] = height; 
                    Eigen::Vector3i temp_index = m_globalMap->coord2gridIndex(temp_coord);
                    
                    (*m_globalMap->GridNodeMap)[_X][_Y]->m_index = temp_index;
                    neighborPtrSets.push_back((*m_globalMap->GridNodeMap)[_X][_Y]);
                    /* 计算边迹代价 */
                    //中心的距离
                    edgeCostSets.push_back(pow((i) * (i) + (j) * (j), 0.5)); 
                                                  
                }
            }
        }
    }
}


// /**
//  * @brief	用点云坐标生成栅格中心点坐标（相当于降采样）后发给rviz以显示出来
//  *          （二维规划下只取指定高度范围的点云）
//  * @param
//  */
// void AstarSearch::visGridMap() {
// //    ROS_WARN("start publish GridMap");
//     pcl::PointCloud <pcl::PointXYZRGB> cloud_vis;
//     sensor_msgs::PointCloud2 map_vis;
//     pcl::PointXYZRGB pt;

//     for (int i = 0; i < GLX_SIZE; i++) {
//         for (int j = 0; j < GLY_SIZE; j++) {
//             for (int k = 0; k < GLZ_SIZE; k++) {
//                 Eigen::Vector3i temp_grid(i, j, 0);



//                 if (global_map->isOccupied(temp_grid, false))
//                 {
//                     Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                     pt.x = temp_pt(0);
//                     pt.y = temp_pt(1);
//                     pt.z = 0.0;
//                     pt.r = 255.0;
//                     pt.g = 255.0;
//                     pt.b = 255.0;
//                     cloud_vis.points.push_back(pt);

//                 }
//                     // if (Global::astar_path_finder->getHeight(i, j, k) > -0.99)
//                 if(global_map->GridNodeMap[i][j]->visibility > 0.1)
//                 {
//                     Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                     pt.x = temp_pt(0);
//                     pt.y = temp_pt(1);
//                     pt.z = 0.0;
//                     pt.r = 255 - global_map->GridNodeMap[i][j]->visibility * 12.0;
//                     pt.g = 0.0;
//                     pt.b = global_map->GridNodeMap[i][j]->visibility * 12.0;
//                     cloud_vis.points.push_back(pt);
//                 }


//             }
//         }
//     }


//     /* 将障碍物的PCL点云数据类型转换为ROS点云通信数据类型*/
//     pcl::toROSMsg(cloud_vis, map_vis);

//     /* 设置坐标系的名称 */
//     map_vis.header.frame_id = "world";

//     /* 发布障碍物点云到rviz中 */
//     grid_map_vis_pub.publish(map_vis);
//     int temp = 1;
//     while (temp--) {
//         pcl::toROSMsg(cloud_vis, map_vis);
//         map_vis.header.frame_id = "world";
//         grid_map_vis_pub.publish(map_vis);
//     }
// }

// /**
//  * @brief	用点云坐标生成栅格中心点坐标（相当于降采样）后发给rviz以显示出来
//  *          （二维规划下只取指定高度范围的点云）
//  * @param
//  */
// void AstarSearch::visLocalGridMap(const pcl::PointCloud <pcl::PointXYZ> &cloud, const bool swell_flag) {
//     pcl::PointCloud <pcl::PointXYZ> cloud_vis;
//     sensor_msgs::PointCloud2 map_vis;
//     pcl::PointXYZ pt;
//     //如果不膨胀
//     if (!swell_flag) {
//         for (int idx = 0; idx < (int) cloud.points.size(); idx++) {

//             pt = cloud.points[idx];
//             //转化成栅格中心的点云坐标
//             Eigen::Vector3d cor_round = coordRounding(Eigen::Vector3d(pt.x, pt.y, pt.z));
//             pt.x = cor_round(0);
//             pt.y = cor_round(1);
//             pt.z = cor_round(2);
//             cloud_vis.points.push_back(pt);
//         }
//     } else {
//         for (int i = 0; i < GLX_SIZE; i++) {
//             for (int j = 0; j < GLY_SIZE; j++) {
//                 for (int k = 0; k < GLZ_SIZE; k++) {
//                     Eigen::Vector3i temp_grid(i, j, k);
//                     //查看是否被占据
//                     if (isLocalOccupied(temp_grid)) {
//                         Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                         pt.x = temp_pt(0);
//                         pt.y = temp_pt(1);
//                         pt.z = global_map->getHeight(i, j) * 1;;
//                         cloud_vis.points.push_back(pt);
//                     }
//                     //存在第二高度且被占据
//                     if(global_map->GridNodeMap[i][j]->exist_second_height == true && global_map->GridNodeMap[i][j]->second_local_occupancy == true)
//                     {
//                         Eigen::Vector3d temp_pt = gridIndex2coord(temp_grid);
//                         pt.x = temp_pt(0);
//                         pt.y = temp_pt(1);
//                         pt.z = 2.0;
//                         cloud_vis.points.push_back(pt);
//                     }
//                 }
//             }
//         }
//     }



//     /* 设置点云的规则（无序、坐标值有限） */
//     cloud_vis.width = cloud_vis.points.size();
//     cloud_vis.height = 1;
//     cloud_vis.is_dense = true;
// //    ROS_WARN("local point size is  %d ", (int)cloud_vis.points.size());

//     /* 将障碍物的PCL点云数据类型转换为ROS点云通信数据类型*/
//     pcl::toROSMsg(cloud_vis, map_vis);
//     /* 设置坐标系的名称 */
//     map_vis.header.frame_id = "world";
//     /* 发布障碍物点云到rviz中 */
//     local_grid_map_vis_pub.publish(map_vis);
//     pcl::toROSMsg(cloud_vis, map_vis);
//     map_vis.header.frame_id = "world";
// }
