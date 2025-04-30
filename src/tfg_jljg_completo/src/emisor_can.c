/**
 * Nodo ROS 2 que escucha comandos de ROS y los transmite por el bus CAN a través de la interfaz can0.
 * Se encarga de manejar los motores, dirección, giro y arranque/parada del sistema.
 *
 * Publica por CAN en los IDs 0x201, 0x202 (dirección), 0x301, 0x302 (velocidades), 0x601, 0x602 (control/arranque).
 */

// =======================
//        INCLUDES
// =======================

// Librerías estándar de C
#include <stdio.h>          // Entrada/salida estándar (printf, perror...)
#include <stdlib.h>         // Funciones útiles (malloc, free, exit...)
#include <string.h>         // Manipulación de cadenas (strcpy, memset...)
#include <unistd.h>         // Funciones del sistema (read, write...)
#include <errno.h>          // Manejo de errores mediante errno

// Librerías para la configuración y uso de sockets CAN
#include <net/if.h>         // Información sobre interfaces de red (e.g., can0)
#include <sys/ioctl.h>      // Control de dispositivos (ioctl)
#include <sys/socket.h>     // API para trabajar con sockets
#include <linux/can.h>      // Estructuras para mensajes CAN
#include <linux/can/raw.h>  // Tipo de socket CAN_RAW para acceso directo
#include <signal.h>         // Manejo de señales como SIGINT (Ctrl+C)
#include <stdbool.h>        // Soporte para tipo booleano

// Librerías de ROS 2 (RCL y RCLC)
#include "rcl/rcl.h"        // API principal de ROS 2 en C
#include "rclc/rclc.h"      // Extensión client library para C
#include "rclc/executor.h"  // Ejecutores para callbacks de ROS 2

// Tipos de mensajes estándar de ROS 2
#include "std_msgs/msg/int32.h"     // Mensaje entero con signo de 32 bits
#include "std_msgs/msg/u_int8.h"    // Mensaje entero sin signo de 8 bits


// =======================
//   VARIABLES GLOBALES
// =======================

// Descriptor del socket CAN y estructuras de configuración
int socket_can;
struct sockaddr_can addr; // Dirección de socket para interfaz CAN
struct ifreq ifr; // Estructura para obtener índice de interfaz (e.g., can0)

// Control de ejecución: se pone en 0 al recibir SIGINT (Ctrl+C)
volatile sig_atomic_t running = 1;

// Variables que almacenan los comandos actuales y últimos enviados para evitar envíos redundantes
int valor_motor_1 = -1, ultimo_motor_1 = -1; // Velocidad motor 1
int valor_motor_2 = -1, ultimo_motor_2 = -1; // Velocidad motor 2

uint8_t sentido_ackermann = 0x07, last_sentido = 0xFF; // Dirección de movimiento general (adelante o atrás)

uint8_t giro = 0; // Modo de movimiento: 0 = Ackermann, 1 = giro sobre eje
uint8_t giro_izq = 0x07, giro_dcha = 0x07; // Sentidos individuales de cada motor (modo giro)
uint8_t last_giro_izq = 0xFF, last_giro_dcha = 0xFF; // Últimos valores enviados de giro

uint8_t estado_arranque = 0; // Estado actual del arranque (0=nada, 1=start, 2=stop)
uint8_t ultimo_estado_arranque = 0; // Último estado publicado

// =======================
//        FUNCIONES
// =======================

// Manejo de señal SIGINT (Ctrl+C) para terminar el bucle principal
void handle_sigint(int sig) {
  (void) sig; // Ignorar el argumento
  running = 0;
}

/*
 * Envía un mensaje CAN por el socket con un ID y uno o dos bytes de datos.
 * can_id: ID del mensaje CAN (ej. 0x301).
 * data_byte: Byte principal a enviar.
 * add_0x05: Si es true, se añade un segundo byte con valor 0x05. Sirve para las consignas de acelerador
 */
void send_can_frame(uint16_t can_id, uint8_t data_byte, bool add_0x05) {
  struct can_frame frame;
  frame.can_id = can_id;

  if (add_0x05) { // Consignas de aceleracion
    frame.can_dlc = 2;
    frame.data[0] = data_byte;
    frame.data[1] = 0x05;
  } else { // Consignas de control
    frame.can_dlc = 1;
    frame.data[0] = data_byte;
  }

  // Envío del frame por el socket CAN y registro en el terminal
  int nbytes = write(socket_can, & frame, sizeof(struct can_frame));
  if (nbytes < 0) {
    perror("Error enviando CAN frame");
  } else {
    if (add_0x05)
      printf("Enviado CAN ID: %X, Data: %02X 05\n", can_id, data_byte);
    else
      printf("Enviado CAN ID: %X, Data: %02X\n", can_id, data_byte);
  }
}

// =================================
// CALLBACKS PARA SUSCRIPCIONES ROS
// =================================

void cb_motor_1(const void * msgin) {
  valor_motor_1 = ((std_msgs__msg__Int32 * ) msgin) -> data;
}

void cb_motor_2(const void * msgin) {
  valor_motor_2 = ((std_msgs__msg__Int32 * ) msgin) -> data;
}

void cb_sentido_ackermann(const void * msgin) {
  sentido_ackermann = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}

void cb_giro(const void * msgin) {
  giro = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}

void cb_giro_izq(const void * msgin) {
  giro_izq = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}

void cb_giro_dcha(const void * msgin) {
  giro_dcha = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}

void cb_arranque(const void * msgin) {
  estado_arranque = ((std_msgs__msg__UInt8 * ) msgin) -> data;

  // Si se ha recibido comando de arranque, reiniciar los estados anteriores; cierre de contactores
  if (estado_arranque == 1) {
    ultimo_motor_1 = -1;
    ultimo_motor_2 = -1;
    last_sentido = 0xFF;
  }
}

// =======================
// PROCESAMIENTO PRINCIPAL
// =======================

/**
 * Función principal de lógica: compara estados actuales y anteriores y envía por CAN si hubo cambios para no saturar.
 * Maneja lógica de dirección, velocidad y arranque.
 */
void process_can() {
  // Enviar comandos de sentido de giro si está seleccionada la rotación sobre sí mismo
  if (giro == 1) {
    if (giro_izq != last_giro_izq) {
      send_can_frame(0x201, giro_izq, false);
      last_giro_izq = giro_izq;
    }
    if (giro_dcha != last_giro_dcha) {
      send_can_frame(0x202, giro_dcha, false);
      last_giro_dcha = giro_dcha;
    }
  } else {
    // Enviar dirección general en caso de movimiento con Ackermann
    if (sentido_ackermann != last_sentido) {
      send_can_frame(0x201, sentido_ackermann, false);
      send_can_frame(0x202, sentido_ackermann, false);
      last_sentido = sentido_ackermann;
    }
  }

  // Enviar velocidad motor 1 si ha cambiado
  if (valor_motor_1 != ultimo_motor_1) {
    send_can_frame(0x301, (uint8_t)(valor_motor_1 & 0xFF), true);
    ultimo_motor_1 = valor_motor_1;
  }

  // Enviar velocidad motor 2 si ha cambiado
  if (valor_motor_2 != ultimo_motor_2) {
    send_can_frame(0x302, (uint8_t)(valor_motor_2 & 0xFF), true);
    ultimo_motor_2 = valor_motor_2;
  }

  // Si cambia el estado de arranque, enviar secuencia de control
  if (estado_arranque != ultimo_estado_arranque) {
    ultimo_estado_arranque = estado_arranque;

    // Detener ambos motores, necesario para cerrar contactores
    send_can_frame(0x201, 0x00, false);
    send_can_frame(0x202, 0x00, false);

    // Si se arranca, enviar contraseña de desbloqueo y poner velocidades a cero
    if (estado_arranque == 1) {
      struct can_frame frame_pass;
      frame_pass.can_dlc = 8;
      frame_pass.data[0] = 0x2B;
      frame_pass.data[1] = 0x00;
      frame_pass.data[2] = 0x50;
      frame_pass.data[3] = 0x02;
      frame_pass.data[4] = 0xDF;
      frame_pass.data[5] = 0x4B;
      frame_pass.data[6] = 0x00;
      frame_pass.data[7] = 0x00;

      frame_pass.can_id = 0x601;
      write(socket_can, & frame_pass, sizeof(frame_pass));
      printf("Enviando password: CAN ID 601, Data: 2B005002DF4B0000\n");

      frame_pass.can_id = 0x602;
      write(socket_can, & frame_pass, sizeof(frame_pass));
      printf("Enviando password: CAN ID 602, Data: 2B005002DF4B0000\n");

      // Enviar velocidad cero a ambos motores
      struct can_frame frame_zero;
      frame_zero.can_dlc = 2;
      frame_zero.data[0] = 0x00;
      frame_zero.data[1] = 0x00;

      frame_zero.can_id = 0x301;
      write(socket_can, & frame_zero, sizeof(frame_zero));
      printf("Enviando CAN ID: 301, Data: 00 00\n");

      frame_zero.can_id = 0x302;
      write(socket_can, & frame_zero, sizeof(frame_zero));
      printf("Enviando CAN ID: 302, Data: 00 00\n");
    }

    // Enviar comando de arranque (operacional) o parada (preoperacional) según estado
    struct can_frame frame;
    frame.can_dlc = 8;
    frame.data[0] = 0x2F;
    frame.data[1] = 0x00;
    frame.data[2] = 0x28;
    frame.data[3] = 0x00;
    frame.data[4] = (estado_arranque == 1) ? 0x00 : 0x01;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    frame.can_id = 0x601;
    write(socket_can, & frame, sizeof(frame));
    printf("Enviado CAN ID: 601, Data: 2F002800%02X000000\n", frame.data[4]);

    frame.can_id = 0x602;
    write(socket_can, & frame, sizeof(frame));
    printf("Enviado CAN ID: 602, Data: 2F002800%02X000000\n", frame.data[4]);
  }
}

// =========================
//     FUNCIÓN PRINCIPAL
// =========================

int main(int argc, char * argv[]) {
  // Capturar Ctrl+C para salir limpiamente
  signal(SIGINT, handle_sigint);

  // Crear socket CAN
  socket_can = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_can < 0) {
    perror("Error abriendo socket CAN");
    return 1;
  }

  // Asignar interfaz CAN (can0)
  strcpy(ifr.ifr_name, "can0");
  ioctl(socket_can, SIOCGIFINDEX, & ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_can, (struct sockaddr * ) & addr, sizeof(addr)) < 0) {
    perror("Error al hacer bind del socket CAN");
    return 1;
  }

  // Inicializar ROS 2
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rcl_node_t node;
  rclc_support_init( & support, argc, argv, & allocator);
  rclc_node_init_default( & node, "emisor_can", "", & support);

  // Crear suscripciones
  rcl_subscription_t sub_motor_1, sub_motor_2, sub_sentido, sub_giro, sub_giro_izq, sub_giro_dcha, sub_arranque;
  rclc_subscription_init_default( & sub_motor_1, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_1");
  rclc_subscription_init_default( & sub_motor_2, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_2");
  rclc_subscription_init_default( & sub_sentido, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "sentido_ackermann");
  rclc_subscription_init_default( & sub_giro, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro");
  rclc_subscription_init_default( & sub_giro_izq, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro_izq");
  rclc_subscription_init_default( & sub_giro_dcha, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro_dcha");
  rclc_subscription_init_default( & sub_arranque, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "arranque");

  // Executor para manejar callbacks
  rclc_executor_t executor;
  rclc_executor_init( & executor, & support.context, 7, & allocator);
  rclc_executor_add_subscription( & executor, & sub_motor_1, & (std_msgs__msg__Int32) {}, & cb_motor_1, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_motor_2, & (std_msgs__msg__Int32) {}, & cb_motor_2, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_sentido, & (std_msgs__msg__UInt8) {}, & cb_sentido_ackermann, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_giro, & (std_msgs__msg__UInt8) {}, & cb_giro, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_giro_izq, & (std_msgs__msg__UInt8) {}, & cb_giro_izq, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_giro_dcha, & (std_msgs__msg__UInt8) {}, & cb_giro_dcha, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_arranque, & (std_msgs__msg__UInt8) {}, & cb_arranque, ON_NEW_DATA);

  // Bucle principal: ejecuta callbacks y procesa lógica de envío
  while (running) {
    rclc_executor_spin_some( & executor, RCL_MS_TO_NS(100));
    process_can();
  }

  // =======================
  //      FINALIZACIÓN
  // =======================

  // Limpieza final
  close(socket_can);

  // Finalizar nodo ROS
  rcl_node_fini( & node);
  rclc_support_fini( & support);
  return 0;
}
