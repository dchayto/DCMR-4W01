cmake_minimum_required(VERSION 3.8)
project(sensors)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interface REQUIRED)

add_executable(wheel_odom_node src/wheel_odometry.cpp)
ament_target_dependencies(wheel_odom_node rclcpp geometry_msgs interface)

target_include_directories(wheel_odom_node 
  PRIVATE 
	../../../common_include/
)

target_link_libraries(wheel_odom_node "${cpp_typesupport_target}") 

install(TARGETS
	wheel_odom_node
	DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
