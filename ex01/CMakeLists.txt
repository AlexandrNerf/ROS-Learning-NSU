cmake_minimum_required(VERSION 3.5)
project(robot_urdf)

find_package(ament_cmake REQUIRED)

# Установка файлов запуска
install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

