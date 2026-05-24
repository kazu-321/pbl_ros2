cmake_minimum_required(VERSION 3.8)
project(pbl_localization)

find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(tf2_eigen REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pbl_localization::pbl_global_localization"
  EXECUTABLE pbl_global_localization_node
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "pbl_wheel_odometry::pbl_wheel_odometry"
  EXECUTABLE pbl_wheel_odometry_node
)

target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} tf2_eigen)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
