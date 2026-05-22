cmake_minimum_required(VERSION 3.8)
project(pbl_control)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "pbl_control::pbl_control"
    EXECUTABLE pbl_control_node
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)