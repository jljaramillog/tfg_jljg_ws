/**
 * Este nodo se suscribe a los topics joystick_acc, joystick_turn, vmax, sentido y giro.
 *
 * Publica comandos de velocidad (motor_1, motor_2) y de sentido individual para el modo giro (giro_izq, giro_dcha),
 * o un sentido común para Ackermann (sentido_ackermann).
 */

// =======================
//     INCLUDES
// =======================

#include <stdio.h>        // Entrada/salida estándar
#include <math.h>         // Funciones matemáticas

// ROS 2 en C
#include "rcl/rcl.h"        // API principal de ROS 2 en C
#include "rclc/rclc.h"      // Extensión client library para C
#include "rclc/executor.h"  // Ejecutores para callbacks de ROS 2

// Tipos de mensajes estándar
#include "std_msgs/msg/int32.h"     // Mensaje entero con signo de 32 bits
#include "std_msgs/msg/u_int8.h"    // Mensaje entero sin signo de 8 bits


// =======================
//       CONSTANTES
// =======================

// Parámetros físicos del vehículo
#define ANCHOVIA_MM 1640.0 // Distancia entre ruedas izquierda y derecha (mm)
#define DISTANCIAEJES_MM 1830.0 // Distancia entre ejes delantero y trasero (mm)
#define GIRO_MAX_angulo_grad 30.0 // Ángulo máximo del volante (grados)

// =======================
//     VARIABLES ROS
// =======================

// Suscripciones a topics
rcl_subscription_t sub_joystick_acc;
rcl_subscription_t sub_joystick_turn;
rcl_subscription_t sub_vmax;
rcl_subscription_t sub_sentido;
rcl_subscription_t sub_giro;

// Declaración de publishers para publicar en distintos topics
rcl_publisher_t pub_motor_1;
rcl_publisher_t pub_motor_2;
rcl_publisher_t pub_sentido_ackermann;
rcl_publisher_t pub_giro_izq;
rcl_publisher_t pub_giro_dcha;

// Variables globales para los datos recibidos
int joystick_acc = 0;
int joystick_turn = 0;
int valor_vmax = 192;
int sentido = 1;
int giro = 0;

// ============================
//     FUNCIONES AUXILIARES
// ============================

/* Convierte el valor del joystick en velocidad, dependiendo del sentido
 * valor_joystick: Valor del joystick (0–255).
 * sentido_val: 1 si es avance, 0 si es retroceso.
 * Devuelve la velocidad normalizada.
 */
unsigned char mapeo_joystick_velocidad(unsigned char valor_joystick, unsigned char sentido_val) {
  if (sentido_val == 1) { // Adelante
    if (valor_joystick <= 0x7F) return 0x00;
    if (valor_joystick >= 0xFE) return 0xFD;
    return (unsigned char)(((valor_joystick - 0x7F) * 0xFD) / (0xFE - 0x7F));
  } else { // Atrás
    if (valor_joystick <= 0x00) return 0xFD;
    if (valor_joystick >= 0x7F) return 0x00;
    return (unsigned char)(0xFD - ((valor_joystick * 0xFD) / 0x7F));
  }
}

/* Escala de velocidad máxima según el valor de vmax
 * Devuelve el factor de escala para la velocidad según el valor del selector vmax.
 * vmax_val: Valor recibido del selector de velocidad máxima.
 */
double escala_vmax(int vmax_val) {
  switch (vmax_val) {
  case 192:
    return 1.0;
  case 193:
    return 0.85;
  case 194:
    return 0.70;
  case 195:
    return 0.50;
  case 196:
    return 0.35;
  case 197:
    return 0.20;
  default:
    return 1.0;
  }
}

// =======================
//      CALLBACKS ROS
// =======================

void cb_joystick_acc(const void * msgin) {
  joystick_acc = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}
void cb_joystick_turn(const void * msgin) {
  joystick_turn = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}
void cb_vmax(const void * msgin) {
  valor_vmax = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}
void cb_sentido(const void * msgin) {
  sentido = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}
void cb_giro(const void * msgin) {
  giro = ((std_msgs__msg__UInt8 * ) msgin) -> data;
}

// =======================
//     LÓGICA DEL TIMER
// =======================

/* Se ejecuta periódicamente y calcula las velocidades y direcciones
 * Callback del temporizador que ejecuta la lógica de control cada 100ms.
 * Calcula las velocidades motoras y direcciones basándose en el tipo de movimiento (Ackermann o giro sobre eje).
 */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) timer;
  (void) last_call_time;

  std_msgs__msg__Int32 motor_1_msg;
  std_msgs__msg__Int32 motor_2_msg;

  if (giro == 1) {
    // Giro sobre sí mismo: ruedas giran en sentidos opuestos, velocidad obtenida del joystick de giro
    unsigned char sentido_izq, sentido_dcha;
    unsigned char velocidad = 0x00;

    if (joystick_turn > 0x7F) {
      sentido_izq = 0x07; // izquierda adelante
      sentido_dcha = 0x0B; // derecha atrás
      velocidad = mapeo_joystick_velocidad((unsigned char) joystick_turn, 1);
    } else if (joystick_turn < 0x7F) {
      sentido_izq = 0x0B; // izquierda atrás
      sentido_dcha = 0x07; // derecha alante
      velocidad = mapeo_joystick_velocidad((unsigned char) joystick_turn, 0);
    } else {
      sentido_izq = 0x00;
      sentido_dcha = 0x00;
      velocidad = 0x00;
    }

    motor_1_msg.data = velocidad;
    motor_2_msg.data = velocidad;

    // Publicar sentidos individuales
    std_msgs__msg__UInt8 giro_izq_msg = {
      .data = sentido_izq
    };
    std_msgs__msg__UInt8 giro_dcha_msg = {
      .data = sentido_dcha
    };
    rcl_publish( & pub_giro_izq, & giro_izq_msg, NULL);
    rcl_publish( & pub_giro_dcha, & giro_dcha_msg, NULL);

  } else {
    // Movimiento tipo Ackermann: diferente velocidad por rueda en curva

    unsigned char velocidad_izqda_raw = mapeo_joystick_velocidad((unsigned char) joystick_acc, sentido);
    unsigned char velocidad_dcha_raw = mapeo_joystick_velocidad((unsigned char) joystick_acc, sentido);
    double velocidad_izqda = (double) velocidad_izqda_raw;
    double velocidad_dcha = (double) velocidad_dcha_raw;

    // Cálculo del ángulo de dirección
    double giro_volante = (double) joystick_turn;
    double angulo_grad = 0.0;
    double radio_giro = INFINITY;

    if (giro_volante < 0x7F) // giro hacia la izquierda
      angulo_grad = -GIRO_MAX_angulo_grad * ((0x7F - giro_volante) / 127.0); // mapeo
    else if (giro_volante > 0x7F) // giro hacia la derecha
      angulo_grad = GIRO_MAX_angulo_grad * ((giro_volante - 0x7F) / 127.0); // mapeo

    if (angulo_grad != 0.0) {
      double angulo_rad = angulo_grad * M_PI / 180.0;
      radio_giro = fabs(DISTANCIAEJES_MM / tan(angulo_rad));
    }

    // Ajustar velocidades para Ackermann
    if (radio_giro != INFINITY) {
      double Ri = radio_giro - (ANCHOVIA_MM / 2.0);
      double Ro = radio_giro + (ANCHOVIA_MM / 2.0);
      double ratio = Ri / Ro;

      if (giro_volante < 0x7F)
        velocidad_izqda *= ratio;
      else if (giro_volante > 0x7F)
        velocidad_dcha *= ratio;
    }

    // Aplicar limitador vmax
    double escalado = escala_vmax(valor_vmax);
    velocidad_izqda *= escalado;
    velocidad_dcha *= escalado;

    motor_1_msg.data = (int) velocidad_izqda;
    motor_2_msg.data = (int) velocidad_dcha;

    // Publicar sentido
    std_msgs__msg__UInt8 sentido_msg = {
      .data = (sentido == 1) ? 0x07 : 0x0B
    };
    rcl_publish( & pub_sentido_ackermann, & sentido_msg, NULL);
  }

  // Publicar velocidades de motores
  rcl_publish( & pub_motor_1, & motor_1_msg, NULL);
  rcl_publish( & pub_motor_2, & motor_2_msg, NULL);
}

// =========================
//     FUNCIÓN PRINCIPAL
// =========================

int main(int argc, char * argv[]) {
  // Inicializar soporte y nodo ROS
  rcl_allocator_t allocator = rcl_get_default_allocator();
  rclc_support_t support;
  rclc_support_init( & support, argc, argv, & allocator);
  rcl_node_t node;
  rclc_node_init_default( & node, "ackermann", "", & support);

  // Crear publishers
  rclc_publisher_init_default( & pub_motor_1, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_1");
  rclc_publisher_init_default( & pub_motor_2, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "motor_2");
  rclc_publisher_init_default( & pub_sentido_ackermann, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "sentido_ackermann");
  rclc_publisher_init_default( & pub_giro_izq, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro_izq");
  rclc_publisher_init_default( & pub_giro_dcha, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro_dcha");

  // Crear suscripciones
  rclc_subscription_init_default( & sub_joystick_acc, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "joystick_acc");
  rclc_subscription_init_default( & sub_joystick_turn, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "joystick_turn");
  rclc_subscription_init_default( & sub_vmax, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "vmax");
  rclc_subscription_init_default( & sub_sentido, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "sentido");
  rclc_subscription_init_default( & sub_giro, & node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8), "giro");

  // Crear temporizador para el bucle de control
  rcl_timer_t timer;
  rclc_timer_init_default( & timer, & support, RCL_MS_TO_NS(100), timer_callback);

  // Executor para manejar callbacks
  rclc_executor_t executor;
  rclc_executor_init( & executor, & support.context, 7, & allocator);
  rclc_executor_add_subscription( & executor, & sub_joystick_acc, & (std_msgs__msg__UInt8) {}, & cb_joystick_acc, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_joystick_turn, & (std_msgs__msg__UInt8) {}, & cb_joystick_turn, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_vmax, & (std_msgs__msg__UInt8) {}, & cb_vmax, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_sentido, & (std_msgs__msg__UInt8) {}, & cb_sentido, ON_NEW_DATA);
  rclc_executor_add_subscription( & executor, & sub_giro, & (std_msgs__msg__UInt8) {}, & cb_giro, ON_NEW_DATA);
  rclc_executor_add_timer( & executor, & timer);

  // Bucle principal del nodo
  while (true) {
    rclc_executor_spin_some( & executor, RCL_MS_TO_NS(100));
  }

  return 0;
}
