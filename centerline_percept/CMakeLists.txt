cmake_minimum_required(VERSION 3.5)
project(centerline_percept)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hobot_cv REQUIRED)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  /usr/hobot/lib
)

add_executable(centerline_detector src/centerline_detector.cpp)

ament_target_dependencies(
  centerline_detector
  rclcpp
  dnn_node
  std_msgs
  sensor_msgs
  hbm_img_msgs
  ai_msgs
  geometry_msgs
  hobot_cv
)

target_link_libraries(centerline_detector
  hbmem
)

target_include_directories(centerline_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS centerline_detector
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/
    DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
