cmake_minimum_required(VERSION 3.8)
project(tf_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(tf_listener_test_executable tf_listener_test)
set(tf_listener_test_library ${tf_listener_test_executable}_core)
set(dependencies
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
)

include_directories(
  include
)

add_executable(${tf_listener_test_executable}
  src/tf_listener_test_node.cpp
  src/tf_listener_test_component.cpp
)
add_library(${tf_listener_test_library} SHARED
  src/tf_listener_test_component.cpp
)
ament_target_dependencies(${tf_listener_test_executable}
  ${dependencies}
)
ament_target_dependencies(${tf_listener_test_library}
  ${dependencies}
)
rclcpp_components_register_nodes(${tf_listener_test_library}
  "tf_test::TfListenerTest"
)

install(TARGETS ${tf_listener_test_executable}
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${tf_listener_test_library}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)

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

ament_export_include_directories(include)
ament_export_libraries(${tf_listener_test_library})
ament_export_dependencies(${dependencies})
ament_package()
