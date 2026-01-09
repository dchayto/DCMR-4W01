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
target_link_libraries(due_interface_node PUBLIC
	rclcpp::rclcpp
	nav_msgs::msg::Odometry
)

target_include_directories(due_interface_node PRIVATE 
	../../../common_include/
)

rosidl_generate_interfaces(${PROJECT_NAME}
	"../../../msg/Wheelspeed.msg"
}

install(TARGETS
	due_interface_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_package()
