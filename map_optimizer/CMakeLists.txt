cmake_minimum_required(VERSION 2.8)
project(map_optimizer)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")
find_package(catkin REQUIRED nav_msgs openslam_gmapping roscpp rostest tf rosbag_storage visualization_msgs laser_geometry map_optimizer_msg interactive_markers)

find_package(Boost REQUIRED signals)

find_package(PCL 1.2 REQUIRED)

#############################
# Eigen ####################
#############################
find_package(Eigen)
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})

# g2o #######################
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
link_directories(${G2O_LIBRARY_DIRS})
link_libraries(${G2O_LIBRARIES})
message("G2o-libraries " ${G2O_LIBRARIES})

include_directories(external/ /usr/include/suitesparse/)

include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

include_directories(src)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -ggdb3 -march=native")

catkin_package()



add_executable(map_optimizer src/graph_manager.cpp src/map_optimizer.cpp src/main.cpp)
target_link_libraries(map_optimizer ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${G2O_LIBRARIES} cxsparse)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(map_optimizer ${catkin_EXPORTED_TARGETS})
endif()



install(TARGETS map_optimizer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
