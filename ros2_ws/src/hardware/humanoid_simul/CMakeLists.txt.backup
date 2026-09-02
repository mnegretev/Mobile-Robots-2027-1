cmake_minimum_required(VERSION 3.8)
project(humanoid_simul)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(humanoid_simul src/humanoid_simul.cpp)
target_include_directories(humanoid_simul PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(humanoid_simul PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(humanoid_simul mujoco glfw)  
ament_target_dependencies(
  humanoid_simul
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
)

add_executable(basic_mujoco src/basic_mujoco.cpp)
target_include_directories(basic_mujoco PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(basic_mujoco PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(basic_mujoco mujoco glfw)  
# ament_target_dependencies(
#   humanoid_simul
#   "rclcpp"
# )

install (
  DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS humanoid_simul
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS basic_mujoco
  DESTINATION lib/${PROJECT_NAME})


ament_package()
