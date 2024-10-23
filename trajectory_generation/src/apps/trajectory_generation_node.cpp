#include"iostream"
#include"replan_fsm.hpp"
#include"ros/ros.h"



int main(int argc,char** argv){
    
    ros::init(argc,argv,"trajectory_generation");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);
    ros::NodeHandle nh("~");

    ReplanFSM roboReplan;
    roboReplan.init(nh);

    ros::Duration(4.0).sleep();
    ROS_WARN("start planning");
    roboReplan.plannerManager->m_astar_path_finder->visGridMap();

    ros::spin();



    return 0;
}