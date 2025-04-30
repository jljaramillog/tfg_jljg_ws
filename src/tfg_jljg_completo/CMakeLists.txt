# Versión mínima de CMake para este proyecto
cmake_minimum_required(VERSION 3.8)

# Nombre del proyecto
project(tfg_jljg_completo)

# =============================================
#        ENCONTRAR PAQUETES DE ROS 2
# =============================================

# CMake busca e incluye los módulos de ROS 2 necesarios
find_package(ament_cmake REQUIRED)  # Sistema de construcción de ROS 2
find_package(rcl REQUIRED)          # Cliente base de ROS 2
find_package(rclc REQUIRED)         # Cliente ligero en C (usado para microcontroladores y bajo nivel)
find_package(std_msgs REQUIRED)     # Tipos de mensajes estándar (UInt8, Int32...)

# =============================================
#       INCLUIR DIRECTORIOS DE CABECERAS
# =============================================

# Permite incluir archivos del directorio "include" en los nodos
include_directories(include)

# =============================================
#     DEFINICIÓN DE EJECUTABLES ROS 2
# =============================================

# Compilar los nodos a partir de los archivos fuente .c
add_executable(ackermann src/ackermann.c)
add_executable(receptor src/receptor.c)
add_executable(emisor_can src/emisor_can.c)

# =============================================
#     ENLACE CON DEPENDENCIAS DE ROS 2
# =============================================

# Asociar las librerías necesarias a cada ejecutable
# (esto resuelve símbolos de ROS, tipos de mensaje, etc.)

ament_target_dependencies(ackermann rclc rcl std_msgs)
ament_target_dependencies(receptor rclc rcl std_msgs)
ament_target_dependencies(emisor_can rcl rclc std_msgs)

# Enlazar librería matemática para cálculos trigonométricos en ackermann.c (tan, sin, cos...)
target_link_libraries(ackermann m)

# =============================================
#          INSTALACIÓN DE EJECUTABLES
# =============================================

# Define qué ejecutables se instalarán y dónde se colocarán
install(TARGETS
  ackermann
  receptor
  emisor_can
  DESTINATION lib/${PROJECT_NAME})  # lib/tfg_jljg_completo

# Instalar la carpeta "launch/" con los archivos de lanzamiento ROS 2
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# =============================================
#       FINALIZAR DEFINICIÓN DEL PAQUETE
# =============================================

# Marca el final del paquete para ROS 2
ament_package()
