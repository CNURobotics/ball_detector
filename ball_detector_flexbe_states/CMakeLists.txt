cmake_minimum_required(VERSION 3.5)
project(ball_detector_flexbe_states)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

INSTALL(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY test
        DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
