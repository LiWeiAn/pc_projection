cmake_minimum_required(VERSION 3.5)
project(pc_projection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS)
find_package(ament_cmake REQUIRED)
#find_package(Boost COMPONENTS system REQUIRED)
#find_package(PCL COMPONENTS common REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

set(dependencies
  pcl_conversions
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  EIGEN3
  PCL
)



add_executable(pc_proj_node src/pc_projection.cpp)
add_executable(ex_proj_node src/ex_projection.cpp)
target_link_libraries (pc_proj_node ${PCL_LIBRARIES})
target_link_libraries (ex_proj_node ${PCL_LIBRARIES})
ament_target_dependencies(pc_proj_node
  pcl_conversions
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  EIGEN3
  PCL
  ${dependencies}
)
ament_target_dependencies(ex_proj_node
  pcl_conversions
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  EIGEN3
  PCL
  ${dependencies}
)

install(TARGETS pc_proj_node
  DESTINATION lib/${PROJECT_NAME})
  install(TARGETS ex_proj_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        launch 
        params 
        DESTINATION share/${PROJECT_NAME})




if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_dependencies(${dependencies})
ament_package()
