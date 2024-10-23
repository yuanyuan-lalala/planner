#include "topo_searcher.hpp"

void TopoSearcher::init(ros::NodeHandle &nh, std::shared_ptr<GlobalMap> &global_map)
{
    m_graph.clear();
    m_eng = std::default_random_engine(m_rd());
    m_rand_pos = std::uniform_real_distribution<double>(0.0, 1.0);
    max_sample_num = 1000;

    m_sample_inflate(0) = 2.0;//这意味着采样区域在 x 方向上扩展了两倍。
    m_sample_inflate(1) = 7.0;//采样区域在 y 方向上扩展了七倍

    global_map_ = global_map;
}

void TopoSearcher::createGraph(Eigen::Vector3d start, Eigen::Vector3d end)
{
    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();
    //    std::vector<GraphNode::Ptr> graph_temp;
    //    m_graph.swap(graph_temp);
    for(int i = 0; i < m_graph.size(); i++)
    {  /// 注意shared_ptr引用计数问题
        m_graph[i]->neighbors.clear();
        m_graph[i]->neighbors_but_noconnected.clear();
    }
    m_graph.clear();
    //起点和终点为守卫点
    GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
    GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));
    m_graph.push_back(start_node);
    m_graph.push_back(end_node);
    int node_id = 1;

    for(int i = 0; i < global_map_->topo_keypoint.size(); i++)  // 只对关键点(路口点)进行采样
    {
        double min_distance = 0.0;
        int exist_unvisiual = 0;
        Eigen::Vector3d pt = global_map_->topo_keypoint[i];
        std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);

        if(visib_guards.size() < 1) { // 这个采样点是不可见的，所以直接建立守卫点
            GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
            m_graph.push_back(guard);

        }else {
            if(min_distance > 2.0){
                GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
                m_graph.push_back(guard);
            }
        }
    }
    int sample_num = 0;
    Eigen::Vector3d pt;
    ROS_DEBUG("[Topo]: m_graph before num: %d", m_graph.size());

    while(sample_num < max_sample_num)
    {  // 开始迭代
        double min_distance = 0.0;
        int exist_unvisiual = 0;
        pt = getSample();  /// 返回的采样点

        Eigen::Vector3i pt_idx = global_map_->coord2gridIndex(pt);

        ++sample_num;
        std::vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt, min_distance, exist_unvisiual);   /// 根据采样点找可见的守卫点

        if(visib_guards.size() < 1) { // 这个采样点是不可见的，所以直接建立守卫点
            GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
            m_graph.push_back(guard);
        }
        else if (visib_guards.size() == 2){  // 这个采样点可以被两个守卫点看到，建立连接点
            checkHeightFeasible(visib_guards[0], visib_guards[1], pt, node_id);
        }
        else if(visib_guards.size() == 1){  // 当只能被一个守卫点看到时，需要判断是否可以建立连接点
            if(exist_unvisiual > 2){  // 这个的意义在于如果迭代到最后有很多点可见的话这个点就不是困难点，没有意义直接扔,阈值太小的话会多很多没必要的警戒点导致topo路径很奇怪
                continue;
            }
            if(min_distance < 0.5){
                continue;
            }
            int obs_num = 0;

            double start_x = pt.x();
            double start_y = pt.y();
            double start_z = pt.z();
            for (int i = 0; i < 40; i++){  // 找0.5m内的占用栅格数量
                //0.5范围内的圆
                double edge_x = start_x + sin(i * M_PI_4 /5) * 0.5;
                double edge_y = start_y + cos(i * M_PI_4/ 5) * 0.5;
                double edge_z = start_z;

                int pt_idx, pt_idy, pt_idz;
                global_map_->coord2gridIndex(edge_x, edge_y, edge_z, pt_idx, pt_idy, pt_idz);
                if (global_map_->isOccupied(pt_idx, pt_idy, pt_idz, false)){
                    obs_num ++;
                }
            }

            if(obs_num >= 10){  // 如果周围的障碍物较多则直接建立守卫点，解决困难样本
                int idx, idy, idz;
                global_map_->coord2gridIndex(pt.x(), pt.y(), pt.z(), idx, idy, idz);
                if(global_map_->GridNodeMap[idx][idy]->exist_second_height_){
                    pt.z() = 0.08;
                }
                GraphNode::Ptr guard = GraphNode::Ptr (new GraphNode(pt, GraphNode::Guard, ++node_id));
                m_graph.push_back(guard);
            }
        }
    }
    t2 = ros::Time::now();
    std::vector<std::vector<Eigen::Vector3d>> temp;  // 搜索可行路径并进行排序，找到最佳路径
    temp = searchPaths();
    //checkDistanceFinalPath();

    ROS_DEBUG("[Topo]: dijkstra time: %f", (ros::Time::now() - t2).toSec());
    ROS_DEBUG("[Topo]: topo search time: %f", (ros::Time::now() - t1).toSec());
    ROS_DEBUG("[Topo]: m_graph num: %d", m_graph.size());
}


std::vector<std::vector<Eigen::Vector3d>> TopoSearcher::searchPaths(int node_id)
{
    raw_paths.clear();
//    std::vector<GraphNode::Ptr> visited;
//    visited.push_back(m_graph[0]);
//    depthFirstSearch(visited);  // 进行深度优先搜索得到部分可行路径
    DijkstraSearch(node_id);  // 改为dijkstra  默认为1

    return raw_paths;
}

void TopoSearcher::DijkstraSearch(int node_id){
    min_path.clear();//最短路径
    std::vector<double> minDist(m_graph.size() + 1, 100000.0);//每个节点到原点的距离向量
    std::vector<bool> visited(m_graph.size() + 1, false);//是否访问过

    Eigen::Vector3d temp = {0,0,0};
    //每个节点的父节点及其对应位置坐标
    std::vector<std::pair<int, Eigen::Vector3d>> parent (m_graph.size() + 1, std::make_pair(-1, temp));

    minDist[0] = 0;
    for (int i = 0; i < m_graph.size(); i++) { // 遍历所有节点
        int minVal = 100000.0;
        int cur_id = -1;
        int index = -1;
        Eigen::Vector3d temp_point;

        // 找距离源点最近(minDist最小)的id点
        for(int j = 0; j < m_graph.size(); j++){
            if(!visited[j] && minDist[j] < minVal){
                minVal = minDist[j];
                cur_id = j;
                temp_point = m_graph[j]->pos;
            }
        }
        // 如果在第一轮循环中（i == 0）没有找到连接的点，检查起点 m_graph[0] 是否有邻居节点。
        // 如果没有连接的邻居节点，则返回错误，表示没有路径。
        if(i == 0 && cur_id < 0){
            if(m_graph[0]->neighbors.size() > 0){
                cur_id = 0;
                index = 0;
                temp_point = m_graph[i]->pos;
            }else{
                ROS_ERROR("[Topo Search] No Path, No point connect to start point");
                return;
            }
        }

        if(cur_id < 0){
            continue;
        }
        visited[cur_id] = true;
        for(int j = 0; j < m_graph[cur_id]->neighbors.size(); j++){
            int id_temp = m_graph[cur_id]->neighbors[j]->m_id;
            if(!visited[id_temp] && minDist[cur_id] + (m_graph[cur_id]->neighbors[j]->pos - temp_point).norm() < minDist[id_temp])
            {
                minDist[id_temp] = minDist[cur_id] + (m_graph[cur_id]->neighbors[j]->pos - temp_point).norm();
                parent[id_temp].first = cur_id;
                parent[id_temp].second = temp_point;
            }
        }
    }
    int index = node_id;  // TODO 这里根据终点的index进行更改变化
    if(minDist[index] > 10000.0){
        if(node_id == 1){
            ROS_ERROR("[Topo Search] No Path, No point connect to end point");
        }
        return;
    }
    min_path.push_back(m_graph[index]->pos);
    while(1){
        min_path.push_back(parent[index].second);
        index = parent[index].first;
        if(index == 0){
            break;
        }
    }
    std::reverse(min_path.begin(), min_path.end());
}

/**
     * @brief 采样点的生成
     * @param
     * @return 采样点
*/
Eigen::Vector3d TopoSearcher::getSample()
{
    Eigen::Vector3d pt;
    if(m_rand_pos(m_eng) < 0.1){  // 10%的概率采样附近的点
        for(int i = 0; i < 4 ; i++){
            int x_i = int(m_rand_pos(m_eng) * 120);//0-120随机数
            int y_i = int(m_rand_pos(m_eng) * 120);
            pt.x() = global_map_->odom_position.x() + (x_i - 60) * 0.1;//当前位置的 [-6, 6] 米范围内。
            pt.y() = global_map_->odom_position.y() + (y_i - 60) * 0.1;
            Eigen::Vector3i pt_idx = global_map_->coord2gridIndex(pt);
            pt.z() = global_map_->getHeight(pt_idx.x(), pt_idx.y());
            if(!global_map_->isOccupied(pt_idx, false)) {
                return pt;
            }
        }
    }
    //90%全图采点
    // 如果没有进入附近点采样的逻辑（即在 90% 的情况下），函数会从全局采样点集合 topo_sample_map 中随机选取一个点。
    // topo_sample_map：这是一个预先定义好的全局采样点列表，包含了地图中已经确定的有效采样点。
    int index = int(m_rand_pos(m_eng) * global_map_->topo_sample_map.size());
    pt = global_map_->topo_sample_map[index];
    return pt;
}


/**
     * @brief 根据采样点找可见的守卫点
     * @param pt 采样点
     * @param dis_temp 采样点到最近的守卫点的距离
*/
std::vector<GraphNode::Ptr> TopoSearcher::findVisibGuard(Eigen::Vector3d pt, double &dis_temp, int &exist_unvisiual){
    std::vector<GraphNode::Ptr> visib_guards;
    Eigen::Vector3d pt_temp;
    double temp_x = pt.x();
    double temp_y = pt.y();
    double min_distance = 10000.0;
    int visib_num = 0;
    //遍历所有的守卫点
    for(std::vector<GraphNode::Ptr>::iterator iter = m_graph.begin(); iter != m_graph.end(); ++iter){
        //计算距离
        double distance = std::sqrt(pow( temp_x - (*iter)->pos.x(), 2) +
                                    pow( temp_y - (*iter)->pos.y(), 2));  // 计算用于找到距离pt最近的点

        if((*iter)->type_ == GraphNode::Connector){  // 如果这个点距离太远或者是连接点就跳过
            continue;
        }
        //不能离采样点太远
        if(distance > 5.0){
            continue;
        }
        //更新最小距离
        if(distance < min_distance){
            min_distance = distance;
        }
        
        if(lineVisib(pt, (*iter)->pos, 0.2, pt_temp, 0)){
            visib_guards.push_back((*iter));
            ++visib_num;
            if (visib_num == 2)  // 如果存在两个可见的守卫点，判断是否需要连接，需要就直接break，否则扔掉其中一个点继续迭代，并把exist_unvisiual+1
            {
                bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
                if(need_connect){
                    break;
                }else{
                    visib_guards[0] = visib_guards[1];
                    visib_guards.pop_back();
                    visib_num --;
                    exist_unvisiual ++;
                }
            }
        }
    }
    dis_temp = min_distance;
    return visib_guards;
}

void TopoSearcher::checkHeightFeasible(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt, int &node_id)
{   // 检查两个守卫点之间是否可以建立连接点,进行双向与单向链接
    int direction;

    GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));  //连接点

    if(heightFeasible(g1->pos, pt, direction)){  // 检查：g1守卫与pt是否高度可通行，可以则建立连接，否则建立单向连接或者不连接，g2同理
        //建立双向连接
        g1->neighbors.push_back(connector);
        connector->neighbors.push_back(g1);
    }else{
        //建立单向连接
        //连接点在下方
        if(direction == 1){
            g1->neighbors.push_back(connector);
            connector->neighbors_but_noconnected.push_back(g1);
        }else if(direction == 2){
            //连接点在上方
            g1->neighbors_but_noconnected.push_back(connector);
            connector->neighbors.push_back(g1);
        }
    }
    if(heightFeasible(g2->pos, pt, direction)){
        g2->neighbors.push_back(connector);
        connector->neighbors.push_back(g2);
    }else{
        if(direction == 1){
            g2->neighbors.push_back(connector);
            connector->neighbors_but_noconnected.push_back(g2);

        }else if(direction == 2){
            g2->neighbors_but_noconnected.push_back(connector);
            connector->neighbors.push_back(g2);
        }
    }
    m_graph.push_back(connector);
}

bool TopoSearcher::heightFeasible(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, int &direction, double step, double thresh)
{
    /**
     * @brief 判断两点之间是否高度可通行
     */
    // p2是connector点, 如果双向可通行，返回true，否则返回false并计算连接方向
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z, p1_x, p1_y, p1_z;
    double distance_thresh;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    p1_x = p1.x();
    p1_y = p1.y();
    p1_z = p1.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();
    double z_offset = p1.z() - p2.z();
    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / step;
    int idx, idy, idz, idx_end, idy_end, idz_end;
    
    global_map_->coord2gridIndex(p2_x, p2_y, p2_z, idx, idy, idz);//起点
    global_map_->coord2gridIndex(p1_x, p1_y, p1_z, idx_end, idy_end, idz_end);//终点
    
    double last_height = p2_z;  ///起点高度
    // 如果两个点都在桥洞区域且高度差很多，说明一个在桥下一个在桥上，直接寄
    if(global_map_->GridNodeMap[idx][idy]->exist_second_height_ &&
        global_map_->GridNodeMap[idx_end][idy_end]->exist_second_height_){
        //高度差过大
        if(abs(p1.z() - p2.z()) > 0.3){
            direction = 0;
            return false;
        }
    }
    // 采样查看是否存在高度差距过大的点
    for(int i = 0; i<(n+1); i++){
        ray_ptx = p2_x + i * step * x_offset / distance;
        ray_pty = p2_y + i * step * y_offset / distance;
        ray_ptz = 0.0;

        int pt_idx, pt_idy, pt_idz;
        global_map_->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, pt_idx, pt_idy, pt_idz);
        double height;
        //不存在第二高度，没有桥洞
        if(!global_map_->GridNodeMap[pt_idx][pt_idy]->exist_second_height_) {
            height = global_map_->getHeight(pt_idx, pt_idy);  //非桥洞区域正常通行
        }else{  // 进入桥洞区域后的高度不再根据bev更新，而是直接等于上一个点的高度。
            height = last_height;
            if(global_map_->GridNodeMap[idx_end][idy_end]->exist_second_height_){
                last_height = p1_z;
            }
        }
        //高度差不能过大
        if(abs(height - last_height) > 0.3){
            direction = 0;
            return false;
        }
        if (abs(height - last_height) > thresh){
            if(height > last_height){
                direction = 1;  // connector处于台阶下方 连接点在下方
            }else{
                direction = 2;  //处于台阶上方  连接点在上方
            }
            return false;
        }
        last_height = height;
    }
    return true;
}
//查看连线是否可通行
bool TopoSearcher::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double thresh, Eigen::Vector3d& pc, int caster_id)
{
    double ray_ptx, ray_pty, ray_ptz;
    double p2_x, p2_y, p2_z;
    double distance_thresh;
    p2_x = p2.x();
    p2_y = p2.y();
    p2_z = p2.z();

    double x_offset = p1.x() - p2.x();
    double y_offset = p1.y() - p2.y();

    double distance = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2));

    int n = std::sqrt(pow(x_offset, 2) + pow(y_offset, 2)) / thresh;
    if(caster_id == 1) {
        distance_thresh = 4.0;
    }

    if(distance < 0.4 && abs(p2_z - p1.z()) > 0.4){  //如果俩点太近且高度差太多直接认为不可见
        return false;
    }
    double last_height = p2_z;

    for(int i = 0; i < n + 1; i++)
    {
        
        ray_ptx = p2_x + i * thresh * x_offset / distance;
        ray_pty = p2_y + i * thresh * y_offset / distance;
        ray_ptz = 0.0;

        int pt_idx, pt_idy, pt_idz;
        bool second_height = false;

        global_map_->coord2gridIndex(ray_ptx, ray_pty, ray_ptz, pt_idx, pt_idy, pt_idz);
        double height = global_map_->getHeight(pt_idx, pt_idy);
        if(global_map_->GridNodeMap[pt_idx][pt_idy]->exist_second_height_){
            height = last_height;
            if(height < 0.4){
                second_height = true;  // 在桥洞区域我们要判断这个点是不是在第二高度上
            }
            last_height = p1.z();
        }
        if(std::abs(height - last_height) > 0.4){  // 这样在只要产生了很大的距离差就直接返回false
            return false;
        }

        if (global_map_->isOccupied(pt_idx, pt_idy, pt_idz, second_height)){
            return false;
        }
        last_height = height;
    }
    return true;
}
  /**
     * @brief 判断两个点是否需要连接，即为两个点不管是单向可连或者双线可连，只要之前相互可见过就不再重新连接
     * @param g1 g2 两个判断的守卫点
     */
bool TopoSearcher::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt){

    for(int i = 0; i < g1->neighbors.size(); i++){
        for(int j = 0; j < g2->neighbors.size(); j++){
            if(g1->neighbors[i]->m_id == g2->neighbors[j]->m_id)  // 只要这俩连接过，我们就认为这段路径就不会重连接了
            {
                return false;
            }
        }
    }

//    for(int i = 0; i < g1->neighbors_but_noconnected.size(); i++){
//        for(int j = 0; j < g2->neighbors_but_noconnected.size(); j++){
//            if(g1->neighbors_but_noconnected[i]->m_id == g2->neighbors_but_noconnected[j]->m_id)
//                // 只要这俩相邻过，我们就认为这段路径就不会重连接了
//            {
//                return false;
//            }
//        }
//    }
//
//    for(int i = 0; i < g1->neighbors.size(); i++){
//        for(int j = 0; j < g2->neighbors_but_noconnected.size(); j++){
//            if(g1->neighbors[i]->m_id == g2->neighbors_but_noconnected[j]->m_id)  // 只要这俩相邻连接过，我们就认为这段路径就不会重连接了
//            {
//                return false;
//            }
//        }
//    }
//
//    for(int i = 0; i < g1->neighbors_but_noconnected.size(); i++){
//        for(int j = 0; j < g2->neighbors.size(); j++){
//            if(g1->neighbors_but_noconnected[i]->m_id == g2->neighbors[j]->m_id)  // 只要这俩**过，我们就认为这段路径就不会重连接了
//            {
//                return false;
//            }
//        }
//    }

    return true;
}


