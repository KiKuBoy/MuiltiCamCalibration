cmake_minimum_required(VERSION 3.16)
project(MuiltiCamCalibration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

# 查找OpenCV库
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS})



# 添加可执行文件
add_executable(MuiltiCamCalibration ${CMAKE_CURRENT_SOURCE_DIR}/src/MuiltiCamCalibration.cpp)
#add_executable(CalibrationTest ${CMAKE_CURRENT_SOURCE_DIR}/src/CalibrationTest.cpp)

# 链接OpenCV库
target_link_libraries(MuiltiCamCalibration ${OpenCV_LIBS}   ${PCL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so ${CERES_LIBRARIES})
#target_link_libraries(CalibrationTest ${OpenCV_LIBS}   ${PCL_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/lib/libyaml-cpp.so ${CERES_LIBRARIES})



