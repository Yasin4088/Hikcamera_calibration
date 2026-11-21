cmake_minimum_required(VERSION 3.8)
project(radar_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
  "msg/TargetArray.msg"
  "srv/AutoAlign.srv"
  DEPENDENCIES std_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

if(cpp_typesupport_target)
  add_library(${PROJECT_NAME}_library INTERFACE)

  target_include_directories(${PROJECT_NAME}_library INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
  )
  target_link_libraries(${PROJECT_NAME}_library INTERFACE "${cpp_typesupport_target}")

  ament_target_dependencies(${PROJECT_NAME}_library INTERFACE
    "std_msgs"
    "geometry_msgs"
    "sensor_msgs"
  )

  install(DIRECTORY include/
    DESTINATION include
  )
  install(
    TARGETS ${PROJECT_NAME}_library EXPORT export_${PROJECT_NAME}
  )

  ament_export_include_directories("include")
  ament_export_targets(export_${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
