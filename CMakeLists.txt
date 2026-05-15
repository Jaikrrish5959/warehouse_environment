cmake_minimum_required(VERSION 3.8)
project(warehouse_env)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  models
  worlds
  launch
  config
  description
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/obstacle_avoidance.py
  scripts/proximity_monitor.py
  scripts/scan_frame_relay.py
  scripts/twist_stamper.py
  scripts/wasd_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
