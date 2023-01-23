cmake_minimum_required(VERSION 3.0.2)
project(rrtx_global_planner)

find_package(catkin REQUIRED
COMPONENTS
  base_local_planner
  costmap_2d
  geometry_msgs
  nav_core
  nav_msgs
  navfn
  pluginlib
  roscpp
  tf
)

# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES rrtx_global_planner
# #  CATKIN_DEPENDS base_local_planner costmap_2d geometry_msgs nav_core nav_msgs navfn pluginlib roscpp tf
# #  DEPENDS system_lib
# )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
        rrtx_global_planner_lib
    CATKIN_DEPENDS
        costmap_2d
        geometry_msgs
        nav_core
        navfn
        nav_msgs
        pluginlib
        roscpp
        tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(rrtx_global_planner_lib src/rrtx_planner.cpp src/rrtx_planner_ros.cpp)
# add_dependencies(rrtx_global_planner_lib ${PROJECT_NAME}_gencfg)
target_link_libraries(rrtx_global_planner_lib ${catkin_LIBRARIES})

install(TARGETS rrtx_global_planner_lib
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES rrtx_global_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)