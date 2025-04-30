# Versión mínima de CMake necesaria
cmake_minimum_required(VERSION 3.8)

# Nombre del proyecto
project(tfg_jljg_inicial)

# =============================================
#        ENCONTRAR PAQUETES DE ROS 2
# =============================================

# Incluye las dependencias básicas de ROS 2 necesarias para compilar nodos en C
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)

# =============================================
#         DEFINICIÓN DE EJECUTABLES
# =============================================

# Nodo básico de envío por CAN
add_executable(envio_can_BASICO src/envio_can_BASICO.c)
target_include_directories(envio_can_BASICO PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(envio_can_BASICO rcl rclc)

# Nodo Ackermann para control diferencial sobre CAN
add_executable(envio_can_ACK src/envio_can_ACK.c)
target_include_directories(envio_can_ACK PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(envio_can_ACK rcl rclc)

# Nodo receptor de mensajes CAN
add_executable(receptor_can src/receptor_can.c)
target_include_directories(receptor_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(receptor_can rcl rclc)

# Enlace con la librería matemática para trigonometría en can_sender_ACK.c
target_link_libraries(envio_can_ACK m)

# =============================================
#         INSTALACIÓN DE EJECUTABLES
# =============================================

# Define qué binarios se instalarán y en qué ruta
install(TARGETS
  envio_can_ACK
  envio_can_BASICO
  receptor_can
  DESTINATION lib/${PROJECT_NAME})

# =============================================
#           FINALIZACIÓN DEL PAQUETE
# =============================================

ament_package()
