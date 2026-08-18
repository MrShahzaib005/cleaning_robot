cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY urdf launch rviz worlds config maps
  DESTINATION share/${PROJECT_NAME}
)

# Python scripts install
install(PROGRAMS
  scripts/ultrasonic_to_pointcloud.py
  scripts/zigzag_cleaner.py
  scripts/wall_follow_cleaner.py
  scripts/random_cleaner.py
  scripts/spiral_cleaner.py
  scripts/pattern_selector.py
  scripts/floor_detector.py
  scripts/vacuum_controller.py
  scripts/dashboard_gui.py
  scripts/create_floor_models.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()