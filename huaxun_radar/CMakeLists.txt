cmake_minimum_required(VERSION 2.8.3)
project(huaxun_radar)

add_compile_options(-std=c++14)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        huaxun_radar_msgs
        rosbag
        pcl_ros
        pcl_conversions
        pcl_msgs
        tf
        tf2
        tf2_eigen
        tf2_ros
        )

find_package(Boost REQUIRED COMPONENTS system filesystem thread)

find_package(PCL REQUIRED)

## Declare a catkin package
catkin_package(
INCLUDE_DIRS lib

)

## Build talker and listener
include_directories(
        ${catkin_INCLUDE_DIRS}
#        ${PCL_INCLUDE_DIRS}
        )


link_directories(lib
        ${catkin_LIB_DIRS}
        )

add_executable(huaxun_radar_node
        src/radar_node.cpp
        src/radar.cpp)


target_include_directories(huaxun_radar_node
        PRIVATE
        src/can
        )

add_dependencies(
    huaxun_radar_node
    huaxun_radar_msgs_generate_messages
)

target_link_libraries(huaxun_radar_node
        PRIVATE
        ${catkin_LIBRARIES}
#        ${PCL_LIBRARIES}
        controlcan)

add_executable(radar_bag_to_pcd
        src/radar_bag_to_pcd.cpp
        )



target_include_directories(
    radar_bag_to_pcd
    PRIVATE
    ${PCL_INCLUDE_DIRS}
)

#link_directories(
#    radar_bag_to_pcd
#    PRIVATE
#    ${PCL_LIBRARY_DIRS}
#)

target_link_libraries(radar_bag_to_pcd
        ${Boost_LIBRARIES}
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
