cmake_minimum_required(VERSION 3.5)
project(barcode_intel)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hbm_img_msgs REQUIRED)


find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBAR REQUIRED zbar)

add_executable(barcode_decoder src/barcode_decoder.cpp)

ament_target_dependencies(
  barcode_decoder
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
  "OpenCV"
  "hbm_img_msgs"
)

target_include_directories(barcode_decoder PRIVATE ${ZBAR_INCLUDE_DIRS})

target_link_libraries(barcode_decoder 
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
)

install(TARGETS barcode_decoder
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
