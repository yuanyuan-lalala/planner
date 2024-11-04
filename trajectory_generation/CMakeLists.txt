cmake_minimum_required(VERSION 3.0.2)
project(planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD_REQUIRED 20)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE Debug)


find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  std_msgs
  message_generation

  roscpp
  rospy
  visualization_msgs
  nav_msgs
  # slaver
  # trajectory_generation
  # sentry_msgs
  ocs2_mpc
  ocs2_core
  ocs2_oc
  ocs2_sqp
  ocs2_robotic_assets
  ocs2_robotic_tools
)
find_package(Glog REQUIRED)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)


add_message_files(DIRECTORY msg FILES trajectoryPoly.msg slaver_speed.msg)
generate_messages(
    DEPENDENCIES
    std_msgs
)
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs message_runtime)



include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/slaver)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_tracking)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/way_points_generator)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/root_solver)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/map)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/optimizer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/visualizer)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_generation/searcher)
include_directories(
    SYSTEM
    include
    third_party
    ${catkin_INCLUDE_DIRS} 
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    # ${Qt5Widgets_INCLUDE_DIRS}
    # ${Qt5Core_INCLUDE_DIRS}
    # ${OGRE_INCLUDE_DIRS}
    ${rviz_INCLUDE_DIRS}
)




file(GLOB_RECURSE ALL_SRCS ALL_SRCS "src/*.cpp")
file(GLOB_RECURSE NODE_SRCS "src/apps/*_node.cpp")
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})


set(ALL_TARGET_LIBRARIES glog::glog 
                        ${PCL_LIBRARIES} 
                        ${OpenCV_LIBS}
                        )

add_executable(trajectory_generation_node src/apps/trajectory_generation_node.cpp ${ALL_SRCS})
add_dependencies(trajectory_generation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_generation_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} )



add_executable(slaver_node src/apps/slaver_node.cpp ${ALL_SRCS})
add_dependencies(slaver_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(slaver_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} )



add_executable(tracking_node src/apps/tracking_node.cpp ${ALL_SRCS})
add_dependencies(tracking_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} )


add_executable(waypoint_generator_node src/apps/waypoint_generator_node.cpp ${ALL_SRCS})
add_dependencies(waypoint_generator_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_generator_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES} )

