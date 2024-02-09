cmake_minimum_required(VERSION 3.8)
project(mandeye_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

message("LASzip library")

add_subdirectory(3rd/LASzip)


add_executable(${PROJECT_NAME} src/save.cpp src/FileSystemClient.cpp src/save_laz.cpp  src/utils/TimeStampReceiver.cpp src/main.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp)
target_include_directories(${PROJECT_NAME} PRIVATE 3rd/json 3rd/LASzip/include include/)
target_link_libraries(${PROJECT_NAME} pthread laszip )

ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

ament_package()
