cmake_minimum_required(VERSION 3.5)
project(camera_aravis)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

unset( CATKIN_INSTALL_INTO_PREFIX_ROOT )

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(Aravis REQUIRED)
find_package(GLIB2 REQUIRED)

include_directories(
  include
  cfg/cpp
  ${Aravis_INCLUDE_DIRS}
  ${GLIB2_INCLUDE_DIRS}
)

add_executable(camnode src/camnode.cpp)
ament_target_dependencies(camnode
  rclcpp
  sensor_msgs
  image_transport
  camera_info_manager
)
target_link_libraries(camnode
  ${Aravis_LIBRARIES} 
  glib-2.0 
  gmodule-2.0 
  gobject-2.0
)

install(TARGETS camnode
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(ament_cmake)

ament_package()
