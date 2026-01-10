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

add_executable(due_interface_node src/due_hw_interface.cpp)
ament_target_dependencies(due_interface_node rclcpp nav_msgs)

target_include_directories(due_interface_node PRIVATE 
	../../../common_include/
)

set(msg_files
	"msg/Wheelspeed.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
	${msg_files}
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(due_interface_node "${cpp_typesupport_target}") 

install(TARGETS
	due_interface_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
