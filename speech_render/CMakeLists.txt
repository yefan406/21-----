cmake_minimum_required(VERSION 3.5)
project(speech_render)

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
find_package(ament_index_cpp REQUIRED)

include_directories(
  include
  wetts/include
)

link_directories(
  wetts/lib
)

set(SOURCES
  src/speech_main.cpp
  src/speech_engine.cpp
  src/utils/alsa_device.cpp
)

add_executable(speech_render_node ${SOURCES})
target_compile_definitions(speech_render_node PRIVATE
  SPEECH_RENDER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
ament_target_dependencies(speech_render_node
  rclcpp
  std_msgs
  ament_index_cpp
)
target_link_libraries(speech_render_node
  tts
  onnxruntime
  asound
)

install(TARGETS
  speech_render_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/wetts/lib/libtts.so
  ${CMAKE_CURRENT_SOURCE_DIR}/wetts/lib/libonnxruntime.so.1.11.1
  DESTINATION lib/)

install(DIRECTORY
  resources
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
