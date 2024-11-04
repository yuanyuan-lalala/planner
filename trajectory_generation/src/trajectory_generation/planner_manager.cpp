#include"planner_manager.hpp"

void PlannerManager::init(ros::NodeHandle& nh,std::shared_ptr<GlobalMap> global_map){
    nh.param("trajectory_generator/reference_v_max", reference_v_max, 2.8);
    nh.param("trajectory_generator/reference_a_max", reference_a_max, 4.2);
    nh.param("trajectory_generator/reference_w_max", reference_w_max, 3.0);
    nh.param("trajectory_generator/reference_desire_speed", reference_desire_speed, 3.0);
    //小陀螺
    nh.param("trajectory_generator/reference_desire_speedxtl", reference_desire_speed_spinning, 2.0);
    nh.param("trajectory_generator/reference_axtl_max", reference_a_spinning_max, 2.0);
    nh.param("trajectory_generator/reference_wxtl_max", reference_w_spinning_max, 2.0);

    nh.param("trajectory_generator/isxtl", is_spinning, false);
    nh.param("trajectory_generator/xtl_flag", spinning_flag, false);
    nh.param("trajectory_generator/occ_file_path", occ_file_path, std::string("occfinal.png"));
    nh.param("trajectory_generator/bev_file_path", bev_file_path, std::string("bevfinal.png"));
    nh.param("trajectory_generator/distance_map_file_path", distance_map_file_path, std::string("distance.png"));

    nh.param("trajectory_generator/map_resolution", map_resolution, 0.1);
    nh.param("trajectory_generator/map_x_size", map_x_size, 28.0);
    nh.param("trajectory_generator/map_y_size", map_y_size, 15.0);
    nh.param("trajectory_generator/map_z_size", map_z_size, 3.0);
    nh.param("trajectory_generator/search_height_min", search_height_min, 0.1);
    nh.param("trajectory_generator/search_height_max", search_height_max, 1.2);
    nh.param("trajectory_generator/search_radius", search_radius, 5.0);

    nh.param("trajectory_generator/robot_radius_dash", robot_radius_dash, 0.1);
    nh.param("trajectory_generator/robot_radius", robot_radius, 0.35);
    nh.param("trajectory_generator/obstacle_swell_flag", obstacle_swell_flag, true);

    nh.param("trajectory_generator/map_lower_point_x", map_lower_point(0), 0.0);
    nh.param("trajectory_generator/map_lower_point_y", map_lower_point(1), 0.0);
    nh.param("trajectory_generator/map_lower_point_z", map_lower_point(2), 0.0);

    ROS_INFO("[Manager Init] map/file_path: %s", map_file_path.c_str());
    ROS_INFO("[Manager Init] map/occ_file_path: %s", occ_file_path.c_str());
    ROS_INFO("[Manager Init] map/bev_file_path: %s", bev_file_path.c_str());
    ROS_INFO("[Manager Init] map/distance_map_file_path: %s", distance_map_file_path.c_str());


    m_eng = std::default_random_engine(m_rd());
    m_rand_pos = std::uniform_real_distribution<double>(-1.0, 1.0);//均匀分布
    sentryColor  = teamColor::RED;

    //resolution 一格多少米 inv_resolution一米多少格
    map_inv_resolution = 1.0 / map_resolution;
    map_upper_point.x() = map_lower_point.x() + map_x_size;
    map_upper_point.y() = map_lower_point.y() + map_y_size;
    map_upper_point.z() = map_lower_point.z() + map_z_size;
    
    grid_max_id_x = (int)map_x_size * map_inv_resolution;
    grid_max_id_y = (int)map_y_size * map_inv_resolution;
    grid_max_id_z = (int)map_z_size * map_inv_resolution;

    initializeManager(nh,global_map);
}



void PlannerManager::initializeManager(ros::NodeHandle& m_nh,std::shared_ptr<GlobalMap> global_map) {
        
        m_global_map.reset();
        m_global_map = global_map;
        if (!global_map) {
            std::cerr << "Error: GlobalMap pointer is null!" << std::endl;
            return;
        }
        m_global_map->initGridMap(m_nh, occ_file_path, bev_file_path, distance_map_file_path, map_resolution,
                                 map_lower_point, map_upper_point, grid_max_id_x, grid_max_id_y, grid_max_id_z,
                                 robot_radius, search_height_min, search_height_max, search_radius);
        m_global_map->setRadiusDash(robot_radius_dash);
        
        m_searcher_manager.reset();
        m_searcher_manager = std::make_shared<SearcherManager>(SearcherManager::AlgorithmType::ASTAR,m_global_map);
        m_optimizer_manager.reset();
        m_optimizer_manager = std::make_shared<OptimizerManager>(OptimizerManager::AlgorithmType::CUBICSPLINE,m_global_map);


}



bool PlannerManager::pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,const Eigen::Vector3d start_vel){
    
    ROS_WARN("[Manager] start point, (x, y): (%.2f, %.2f)", start_pt.x(), start_pt.y());
    ROS_WARN("[Manager] receive target, (x, y): (%.2f, %.2f)", target_pt.x(), target_pt.y());
    
    m_searcher_manager->setPoints(start_pt,target_pt);
    m_searcher_manager->executeSearch();
    unoptimized_path.clear();
    std::vector<Eigen::Vector3d> origin_path;
    if((m_searcher_manager->algorithm->getPath()).size() > 0){
        origin_path = m_searcher_manager->algorithm->getPath();
    }else{
        ROS_ERROR("[Manager PLANNING] Invalid target point,global planning failed");
        m_global_map->resetUsedGrids();
        return false;
    }
    unoptimized_path = m_searcher_manager->smoothTopoPath(origin_path);  // 剪枝优化topo路径
    std::cout << "unoptimized_path size: " << unoptimized_path.size() << std::endl;
    m_global_map->resetUsedGrids();

    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();

    double reference_speed = is_spinning? reference_desire_speed_spinning : reference_desire_speed;
    ROS_WARN("[Manager] reference_speed: (%.2f)", reference_speed);
    
    m_optimizer_manager->setParameters(unoptimized_path, start_vel,reference_speed);
    m_optimizer_manager->executeOptimization();
    
    m_reference_optimizer->setGlobalPath(start_vel, final_path, reference_a_max, reference_speed, is_spinning);
    m_reference_optimizer->getRefTrajectory(ref_trajectory, m_optimizer_manager->optimizer->m_trapezoidal_time);
    m_reference_optimizer->getRefVel(); 
    
    ROS_INFO("[Manager] optimizer generate time: %f", (ros::Time::now() - t1).toSec());

    astar_path = origin_path;
    if(ref_trajectory.size() < 2){
        return false;
    }
    return true;

}