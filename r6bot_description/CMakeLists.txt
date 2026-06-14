cmake_minimum_required(VERSION 3.8)
project(r6bot_description)

find_package(ament_cmake REQUIRED)

# Instala los recursos del paquete (URDF, mallas, launch, rviz)
install(
  DIRECTORY urdf meshes launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
