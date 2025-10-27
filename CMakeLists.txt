cmake_minimum_required(VERSION 3.8)
project(rmf_gz)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY resources/
  DESTINATION share/${PROJECT_NAME}/resources
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(DIRECTORY scripts/
    DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py"
  )
endif()

ament_package()
