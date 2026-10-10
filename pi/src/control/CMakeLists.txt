cmake_minimum_required(VERSION 3.8)
project(control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)

add_executable(due_interface_node src/due_hw_interface.cpp)
target_link_libraries(due_interface_node PUBLIC
	${interface_TARGETS}
	rclcpp::rclcpp
)

add_executable(kb_handler_node src/kb_input_handler.cpp)
target_link_libraries(kb_handler_node PUBLIC
	${geometry_msgs_TARGETS}
	rclcpp::rclcpp
)

add_executable(mecanum_controller_node src/mec_wheel_controller.cpp)
target_link_libraries(mecanum_controller_node PUBLIC
	${geometry_msgs_TARGETS}
	${interface_TARGETS}
	rclcpp::rclcpp
)

target_include_directories(due_interface_node 
  PRIVATE 
	../../../common_include/
)

target_include_directories(mecanum_controller_node
  PRIVATE
  	../../../common_include/
)

target_include_directories(kb_handler_node
  PRIVATE
  	../../../common_include/
)

install(TARGETS
	due_interface_node
	mecanum_controller_node
	kb_handler_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
