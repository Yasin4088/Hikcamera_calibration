cmake_minimum_required(VERSION 3.22)
project(livox_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-O3)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost COMPONENTS date_time filesystem iostreams REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED src/publisher.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN livox_lidar::LidarPublisher
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE)
