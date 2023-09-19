cmake_minimum_required(VERSION 3.0.2)
project(multi_cam_rgb_data)


#add_compile_options(-std=c++11)

set  (CMAKE_BUILD_TYPE "Release")
set  (CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -static-libgcc -static-libstdc++")
set  (CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -pthread -static-libgcc -static-libstdc++")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  std_msgs
  sensor_msgs
  message_filters
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS 
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/gain_data_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
	    ${catkin_LIBRARIES}
		${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        -lopencv_imgcodecs)
