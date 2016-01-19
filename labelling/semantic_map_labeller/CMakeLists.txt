cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_labeller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs pcl_ros qt_build semantic_map metaroom_xml_parser image_transport object_manager)

#set(CMAKE_CXX_FLAGS "-fopenmp -O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "-g -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

link_directories(/opt/ros/hydro/lib/)

rosbuild_prepare_qt4(QtCore QtXml)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS metaroom_xml_parser
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${QT_INCLUDE_DIRS}
)

add_executable(create_int_images  include/labeller.h src/create_int_images.cpp)
add_executable(pcd_from_label  include/labeller.h src/pcd_from_label.cpp)
add_executable(copy_labels  include/labeller.h src/copy_labels.cpp)
add_executable(copy_label_files  include/labeller.h src/copy_label_files.cpp)
add_executable(create_xml_from_labels  include/labeller.h src/create_xml_from_labels.cpp)
add_executable(view_labelled_objects  include/labeller.h src/view_labelled_objects.cpp)
add_executable(export_waypoint_sweeps  include/labeller.h src/export_waypoint_sweeps.cpp)
add_executable(list_waypoint_sweeps  include/labeller.h src/list_waypoint_sweeps.cpp)
add_executable(rebuild_merged_cloud  src/rebuild_merged_cloud.cpp)
add_executable(export_sweeps_and_reduce_int_clouds src/export_sweeps_and_reduce_int_clouds.cpp)
add_executable(rename_label src/rename_label.cpp)


 target_link_libraries(create_int_images
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(pcd_from_label
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(copy_labels
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(copy_label_files
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(create_xml_from_labels
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(view_labelled_objects
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(export_waypoint_sweeps
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

 target_link_libraries(list_waypoint_sweeps
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )
 
target_link_libraries(rebuild_merged_cloud
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

target_link_libraries(export_sweeps_and_reduce_int_clouds
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )

target_link_libraries(rename_label
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${QT_LIBRARIES}
 )


