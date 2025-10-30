cmake_minimum_required(VERSION 3.8)
project(rmf_gz)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

install(
    DIRECTORY launch rviz resources
    DESTINATION share/${PROJECT_NAME}
)
file(GLOB PY_SCRIPTS scripts/*.py)
install(PROGRAMS
  ${PY_SCRIPTS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
