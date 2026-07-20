cmake_minimum_required(VERSION 3.8)
project(mapping_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(random_numbers REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(k_means_solved src/k_means_solved.cpp)
target_include_directories(k_means_solved PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  "/usr/include/eigen3/"
  #${Eigen3_INCLUDE_DIRS}
)
target_compile_features(k_means_solved PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  k_means_solved
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "nav_msgs"
  "graph_msgs"
  "random_numbers"
)

install(TARGETS k_means_solved
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
