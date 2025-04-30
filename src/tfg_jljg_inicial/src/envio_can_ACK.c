/**
 * Este programa en C recibe datos desde el bus CAN y calcula las velocidades de motores 
 * siguiendo una cinemática de tipo Ackermann. Interactúa directamente con la interfaz CAN
 * y envía comandos a dos controladoras de motor (301 y 302) en función del valor del joystick
 * y del estado de avance/retroceso recibido.
 *
 * Entradas desde el bus CAN:
 *  - 0x186 (joystick): Contiene dos bytes relevantes para velocidad y giro.
 *  - 0x201 y 0x202 (estado): Byte único que indica sentido de marcha para motor 1 y 2.
 *
 * Salidas al bus CAN:
 *  - 0x301 y 0x302: Comandos de velocidad (byte 0) y modo (byte 1 = 0x05).
 */

// =======================
//        INCLUDES
// =======================

#include <stdio.h>      // Entrada/salida estándar
#include <stdlib.h>     // Funciones generales (exit, etc.)
#include <string.h>     // Manipulación de cadenas
#include <unistd.h>     // Funciones de POSIX (close)
#include <fcntl.h>      // Control de archivos
#include <sys/ioctl.h>  // Control de dispositivos
#include <linux/can.h>  // Estructura CAN (can_frame)
#include <sys/socket.h> // Funciones de sockets
#include <sys/types.h>  // Tipos de sockets
#include <arpa/inet.h>  // Utilidades de red
#include <net/if.h>     // Información de interfaz de red
#include <signal.h>     // Manejo de señales (Ctrl+C)
#include <math.h>       // Funciones matemáticas

// =======================
//     DEFINICIONES CAN
// =======================

#define INTERFAZ_CAN "can0"           // Nombre de la interfaz CAN

// Identificadores CAN (COB-ID)
#define COBID_JOYSTICK         0x186   // Entrada del joystick
#define COBID_CONTROLADORA_1   0x301   // Motor izquierdo
#define COBID_CONTROLADORA_2   0x302   // Motor derecho
#define COBID_ESTADO_1         0x201   // Estado del motor 1
#define COBID_ESTADO_2         0x202   // Estado del motor 2

// =======================
//   PARÁMETROS FÍSICOS
// =======================

#define DISTANCIAEJES_MM 1830.0           // Distancia entre ejes (mm)
#define ANCHOVIA_MM 1640.0                // Distancia entre ruedas (mm)
#define GIRO_MAX_GRAD 30.0                // Ángulo máximo de giro (grados)
#define GRAD_A_RAD (M_PI / 180.0)         // Conversión de grados a radianes

// =======================
//   VARIABLES GLOBALES
// =======================

unsigned char estado_301 = 0x07;      // Estado actual del motor 1 (por defecto: avance)
unsigned char estado_302 = 0x07;      // Estado actual del motor 2
unsigned char ult_vel_301 = 0xFF;     // Última velocidad enviada al motor 1 (para evitar redundancia)
unsigned char ult_vel_302 = 0xFF;     // Última velocidad enviada al motor 2
int s;                                // Descriptor del socket CAN

// ============================
//     MANEJO DE SEÑALES
// ============================

/**
 * Captura Ctrl+C (SIGINT) y cierra el socket antes de salir.
 */
void cerrar_programa(int signo) {
    printf("\nCerrando programa...\n");
    if (s >= 0) close(s);
    exit(0);
}

// ============================
//    FUNCIONES DEL SOCKET
// ============================

/**
 * Inicializa y enlaza el socket CAN.
 * Devuelve el descriptor del socket o -1 en caso de error.
 */
int open_can_socket() {
    int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock < 0) {
        perror("Error al abrir el socket CAN");
        return -1;
    }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, INTERFAZ_CAN, sizeof(ifr.ifr_name) - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("Error al obtener el índice de la interfaz CAN");
        close(sock);
        return -1;
    }

    struct sockaddr_can addr = { .can_family = AF_CAN, .can_ifindex = ifr.ifr_ifindex };
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error al enlazar el socket CAN");
        close(sock);
        return -1;
    }

    return sock;
}

// ============================
//   FUNCIONES DE MAPEADO
// ============================

/**
 * Convierte el valor del joystick (0–255) en velocidad (0x00–0xFD),
 * teniendo en cuenta el sentido (adelante o atrás).
 */
unsigned char mapeo_joystick_velocidad(unsigned char valor_joystick, unsigned char estado) {
    if (estado == 0x07) { // Adelante
        if (valor_joystick <= 0x7F) return 0x00;
        if (valor_joystick >= 0xFE) return 0xFD;
        return (unsigned char)(((valor_joystick - 0x7F) * 0xFD) / (0xFE - 0x7F));
    } else if (estado == 0x0B) { // Atrás
        if (valor_joystick <= 0x00) return 0xFD;
        if (valor_joystick >= 0x7F) return 0x00;
        return (unsigned char)(0xFD - ((valor_joystick * 0xFD) / 0x7F));
    }
    return 0x00; // Estado desconocido o sin movimiento
}

/**
 * Envía un mensaje CAN si la velocidad ha cambiado respecto a la última enviada.
 */
void send_can_message(int sock, unsigned int cobid, unsigned char velocidad, unsigned char *ult_velocidad) {
    if (*ult_velocidad == velocidad) return; // No enviar si no hay cambio
    *ult_velocidad = velocidad;

    struct can_frame frame = {
        .can_id = cobid,
        .can_dlc = 2,
        .data = { velocidad, 0x05 } // El segundo byte es fijo
    };

    if (write(sock, &frame, sizeof(frame)) == sizeof(frame)) {
        printf("CAN -> 0x%X: [%02X %02X]\n", cobid, frame.data[0], frame.data[1]);
    } else {
        perror("Error al enviar mensaje CAN");
    }
}

// ============================
//   CÁLCULO DE VELOCIDADES
// ============================

/**
 * Realiza el cálculo tipo Ackermann según los valores del joystick y envía las velocidades.
 * joystick_vel: valor de aceleración
 * joystick_turn: valor de giro
 */
void calcular_y_enviar_velocidades(unsigned char joystick_vel, unsigned char joystick_turn) {
    // Mapeo del joystick a velocidad base
    unsigned char base_speed_301 = mapeo_joystick_velocidad(joystick_vel, estado_301);
    unsigned char base_speed_302 = mapeo_joystick_velocidad(joystick_vel, estado_302);
    double vel_izq = (double)base_speed_301;
    double vel_dcha = (double)base_speed_302;

    // Cálculo del ángulo de dirección
    double angulo_grad = 0.0;
    if (joystick_turn < 0x7F)
        angulo_grad = -GIRO_MAX_GRAD * ((0x7F - joystick_turn) / 127.0);
    else if (joystick_turn > 0x7F)
        angulo_grad =  GIRO_MAX_GRAD * ((joystick_turn - 0x7F) / 127.0);

    // Cálculo del radio de giro (Ackermann)
    double radio_giro = INFINITY;
    if (angulo_grad != 0.0) {
        double angulo_rad = angulo_grad * GRAD_A_RAD;
        radio_giro = fabs(DISTANCIAEJES_MM / tan(angulo_rad));
    }

    // Ajuste de velocidades por curva
    if (radio_giro != INFINITY) {
        double Ri = radio_giro - (ANCHOVIA_MM / 2.0);
        double Ro = radio_giro + (ANCHOVIA_MM / 2.0);
        double ratio = Ri / Ro;

        if (joystick_turn < 0x7F)
            vel_izq *= ratio;
        else if (joystick_turn > 0x7F)
            vel_dcha *= ratio;
    }

    // Redondeo y envío
    unsigned char final_301 = (unsigned char)(vel_izq + 0.5);
    unsigned char final_302 = (unsigned char)(vel_dcha + 0.5);

    send_can_message(s, COBID_CONTROLADORA_1, final_301, &ult_vel_301);
    send_can_message(s, COBID_CONTROLADORA_2, final_302, &ult_vel_302);
}

// ============================
//     FUNCIÓN PRINCIPAL
// ============================

int main() {
    signal(SIGINT, cerrar_programa); // Ctrl+C

    s = open_can_socket();
    if (s < 0) return 1;

    struct can_frame frame;

    while (1) {
        int nbytes = read(s, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("Error al leer del bus CAN");
            continue;
        }

        // Procesamiento de mensajes CAN entrantes
        switch (frame.can_id) {
            case COBID_ESTADO_1:
                estado_301 = frame.data[0];
                printf("Estado 301: 0x%02X\n", estado_301);
                break;

            case COBID_ESTADO_2:
                estado_302 = frame.data[0];
                printf("Estado 302: 0x%02X\n", estado_302);
                break;

            case COBID_JOYSTICK:
                printf("Joystick: Vel=0x%02X Giro=0x%02X\n", frame.data[1], frame.data[3]);
                calcular_y_enviar_velocidades(frame.data[1], frame.data[3]);
                break;
        }
    }

    return 0;
}
