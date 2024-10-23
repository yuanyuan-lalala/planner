#include"planner_manager.hpp"

void PlannerManager::init(ros::NodeHandle& nh){
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
    sentryColor = teamColor::RED;

    //resolution 一格多少米 inv_resolution一米多少格
    map_inv_resolution = 1.0 / map_resolution;
    map_upper_point.x() = map_lower_point.x() + map_x_size;
    map_upper_point.y() = map_lower_point.y() + map_y_size;
    map_upper_point.z() = map_lower_point.z() + map_z_size;
    
    grid_max_id_x = (int)map_x_size * map_inv_resolution;
    grid_max_id_y = (int)map_y_size * map_inv_resolution;
    grid_max_id_z = (int)map_z_size * map_inv_resolution;

    m_globalMap.reset(new GlobalMap);
    m_globalMap->initGridMap(nh, occ_file_path, bev_file_path, distance_map_file_path, map_resolution,
                            map_lower_point, map_upper_point, grid_max_id_x, grid_max_id_y, grid_max_id_z,
                            robot_radius, search_height_min, search_height_max, search_radius); 

    m_smoother.reset(new Smoother);
    m_smoother->setGlobalMap(m_globalMap);

    m_referenceSmooth.reset(new ReferenceSmooth);
    m_referenceSmooth->init(m_globalMap);

    m_topoPRM.reset(new TopoSearcher);
    m_topoPRM->init(nh, m_globalMap);

    m_globalMap->setRadiusDash(robot_radius_dash);  // 动态障碍物膨胀半径设置
}

bool PlannerManager::pathFinding(const Eigen::Vector3d start_pt, const Eigen::Vector3d target_pt,const Eigen::Vector3d start_vel){
    ROS_WARN("[Manager] start point, (x, y): (%.2f, %.2f)", start_pt.x(), start_pt.y());
    ROS_WARN("[Manager] receive target, (x, y): (%.2f, %.2f)", target_pt.x(), target_pt.y());
    m_topoPRM->createGraph(start_pt, target_pt);


    optimized_path.clear();
    std::vector<Eigen::Vector3d> origin_path;
    if(m_topoPRM->min_path.size() > 0){
        origin_path = m_topoPRM->min_path;
    }else{
        ROS_ERROR("[Manager PLANNING] Invalid target point，global planning failed");
        m_astar_path_finder->global_map->resetUsedGrids();
        return false;
    }
    optimized_path = m_astar_path_finder->smoothTopoPath(origin_path);  // 剪枝优化topo路径
    std::cout<<"optimized_path size: "<<optimized_path.size()<<std::endl;
    m_astar_path_finder->global_map->resetUsedGrids();

    ros::Time t1, t2;  /// 计算时间
    t1 = ros::Time::now();

    double reference_speed = is_spinning? reference_desire_speed_spinning : reference_desire_speed;
    ROS_WARN("[Manager] reference_speed: (%.2f)", reference_speed);
    
    m_smoother->init(optimized_path, start_vel, reference_speed);
    //final_path = path_smoother->getSamplePath();
    m_smoother->smoothPath();
    m_smoother->pathResample();
    final_path = m_smoother->getPath();
    //    final_path_temp = path_smoother->getSamplePath();
    ROS_INFO("[Manager] optimizer generate time: %f", (ros::Time::now() - t1).toSec());

    m_referenceSmooth->setGlobalPath(start_vel, final_path, reference_a_max, reference_speed, is_spinning);
    m_referenceSmooth->getRefTrajectory(ref_trajectory, m_smoother->m_trapezoidal_time);
    m_referenceSmooth->getRefVel();

    astar_path = origin_path;
    if(ref_trajectory.size() < 2){
        return false;
    }
    return true;
    //reference_path.getRefVel();  // 这个是用来可视化的
    //visReferenceTrajectory(ref_trajectory);
    //visOptGlobalPath(final_path);
}