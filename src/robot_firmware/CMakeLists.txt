cmake_minimum_required(VERSION 3.8)
project(robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(SERIAL libserial)

add_executable(serial_transmitter src/serial_transmitter.cpp)
ament_target_dependencies(serial_transmitter rclcpp std_msgs)
target_include_directories(serial_transmitter PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(serial_transmitter "${SERIAL_LDFLAGS}")

add_executable(serial_receiver src/serial_receiver.cpp)
ament_target_dependencies(serial_receiver rclcpp std_msgs)
target_include_directories(serial_receiver PUBLIC "${SERIAL_INCLUDE_DIRS}")
target_link_libraries(serial_receiver "${SERIAL_LDFLAGS}")

add_executable(mpu6050_driver src/mpu6050_driver.cpp)
ament_target_dependencies(mpu6050_driver rclcpp sensor_msgs)
target_link_libraries(mpu6050_driver i2c)

install(TARGETS 
  serial_transmitter
  serial_receiver
  mpu6050_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION bin 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
