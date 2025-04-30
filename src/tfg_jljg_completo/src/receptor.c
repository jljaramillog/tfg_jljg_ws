/*
 * Nodo ROS 2 que escucha el bus CAN por la interfaz can0 y publica en ROS los valores relevantes
 * para el control del vehículo (joystick, vmax, sentido, arranque, etc.).
 *
 * Publica en los topics: joystick_acc, joystick_turn, vmax, sentido, giro, arranque.
 * Escucha CAN IDs: 0x186 (joystick), 0x286 (control).
 */

// =======================
//       INCLUDES
// =======================

// Librerías estándar de C
#include <stdio.h>          // Entrada/salida estándar (printf, perror...)
#include <stdlib.h>         // Funciones útiles (malloc, free, exit...)
#include <string.h>         // Manipulación de cadenas (strcpy, memset...)
#include <unistd.h>         // Funciones del sistema (read, write...)
#include <errno.h>          // Manejo de errores mediante errno

// Librerías para la configuración y uso de sockets CAN
#include <net/if.h>         // Información sobre interfaces de red (can0, etc.)
#include <sys/ioctl.h>      // Control de dispositivos (ioctl)
#include <sys/socket.h>     // API para trabajar con sockets
#include <linux/can.h>      // Estructuras para mensajes CAN
#include <linux/can/raw.h>  // Tipo de socket CAN_RAW para acceso directo
#include <signal.h>         // Para capturar SIGINT (Ctrl+C)

// Librerías de ROS 2 (RCL y RCLC)
#include "rcl/rcl.h"        // API principal de ROS 2 en C
#include "rclc/rclc.h"      // Extensión client library para C

// Tipos de mensajes estándar de ROS 2
#include "std_msgs/msg/u_int8.h"    // Mensaje entero sin signo de 8 bits


// =======================
//     CONSTANTES
// =======================

// Identificadores CAN (COB-ID) de los dispositivos conectados
#define COB_ID_JOYSTICK 0x186 // Mensajes provenientes de los joysticks
#define COB_ID_CONTROL 0x286 // Mensajes de control (vmax, sentido, giro, arranque)

// =======================
//   VARIABLES GLOBALES
// =======================

volatile sig_atomic_t running = 1; // Control de bucle principal (1 = seguir, 0 = salir)

// =======================
//  MANEJADOR DE SEÑALES
// =======================

// Manejo de señal SIGINT (Ctrl+C) para terminar el bucle principal
void handle_sigint(int sig) {
  (void) sig; // Ignorar el argumento
  running = 0;
}

// =======================
//    FUNCIÓN PRINCIPAL
// =======================

int main(int argc, char * argv[]) {
  // Capturar Ctrl+C para salir limpiamente
  signal(SIGINT, handle_sigint);

  // Estructuras básicas de ROS 2
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;

  // Inicialización del soporte y nodo ROS 2
  rclc_support_init( & support, argc, argv, & allocator);
  rclc_node_init_default( & node, "receptor", "", & support);

  // Declaración de publishers para publicar en distintos topics
  rcl_publisher_t pub_joystick_acc;
  rcl_publisher_t pub_joystick_turn;
  rcl_publisher_t pub_vmax;
  rcl_publisher_t pub_sentido;
  rcl_publisher_t pub_giro;
  rcl_publisher_t pub_arranque;

  // Crear publishers
  rclc_publisher_init_default( & pub_arranque, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "arranque");
  rclc_publisher_init_default( & pub_joystick_acc, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "joystick_acc");
  rclc_publisher_init_default( & pub_joystick_turn, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "joystick_turn");
  rclc_publisher_init_default( & pub_vmax, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "vmax");
  rclc_publisher_init_default( & pub_sentido, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "sentido");
  rclc_publisher_init_default( & pub_giro, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro");

  // =======================
  //  CONFIGURAR SOCKET CAN
  // =======================

  int s;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame frame; // Estructura que representa un mensaje CAN

  // Crear socket CAN tipo RAW
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (s < 0) {
    perror("Error abriendo el socket CAN");
    return 1;
  }

  // Asociar el socket a la interfaz can0
  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, & ifr); // Obtener índice de can0

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  // Vincular socket con la interfaz can0
  bind(s, (struct sockaddr * ) & addr, sizeof(addr));

  printf("Leyendo mensajes CAN en can0...\n");

  // =======================
  //     BUCLE PRINCIPAL
  // =======================

  while (running) {
    // Leer una trama CAN del bus
    int nbytes = read(s, & frame, sizeof(struct can_frame));
    if (nbytes < 0) {
      perror("Error leyendo del socket");
      break;
    }

    // =======================
    //   JOYSTICK: ID 0x186
    // =======================
    if (frame.can_id == COB_ID_JOYSTICK) {
      std_msgs__msg__UInt8 msg_acc;
      std_msgs__msg__UInt8 msg_turn;

      // Extraer aceleración (byte 1) y giro (byte 3)
      msg_acc.data = frame.data[1];
      msg_turn.data = frame.data[3];

      // Publicar valores en ROS
      rcl_publish( & pub_joystick_acc, & msg_acc, NULL);
      rcl_publish( & pub_joystick_turn, & msg_turn, NULL);

      printf("Publicado acelerador=%d, giro=%d\n", msg_acc.data, msg_turn.data);
    }

    // =======================
    //    CONTROL: ID 0x286
    // =======================
    if (frame.can_id == COB_ID_CONTROL) {
      std_msgs__msg__UInt8 msg_vmax;
      std_msgs__msg__UInt8 msg_sentido;
      std_msgs__msg__UInt8 msg_giro;
      std_msgs__msg__UInt8 msg_arranque;

      // Byte 5: vmax
      msg_vmax.data = frame.data[5];

      // Byte 0: byte de control con varios botones, cada uno en un bit
      uint8_t control_byte = frame.data[0];

      // Bit 5: sentido (adelante/atrás)
      msg_sentido.data = (control_byte >> 5) & 0x01;

      // Bit 0: modo giro (0 = Ackermann, 1 = giro sobre eje)
      msg_giro.data = control_byte & 0x01;

      // Publicar vmax, sentido y giro
      rcl_publish( & pub_vmax, & msg_vmax, NULL);
      rcl_publish( & pub_sentido, & msg_sentido, NULL);
      rcl_publish( & pub_giro, & msg_giro, NULL);

      printf("Publicado vmax=%d, sentido=%d, giro=%d\n", msg_vmax.data, msg_sentido.data, msg_giro.data);

      // Bits 1 y 2: control de arranque/parada
      if ((control_byte >> 1) & 0x01) {
        msg_arranque.data = 1; // Start
        rcl_publish( & pub_arranque, & msg_arranque, NULL);
        printf("Publicado arranque: START\n");
      } else if ((control_byte >> 2) & 0x01) {
        msg_arranque.data = 2; // Stop
        rcl_publish( & pub_arranque, & msg_arranque, NULL);
        printf("Publicado arranque: STOP\n");
      }
    }
  }

  // =======================
  //      FINALIZACIÓN
  // =======================

  // Cerrar socket CAN
  close(s);

  // Finalizar nodo ROS
  rcl_node_fini( & node);

  return 0;
}
