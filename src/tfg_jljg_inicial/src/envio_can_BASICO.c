/**
 * Programa para enviar tramas CAN manualmente mediante consola.
 * 
 * El usuario debe introducir los mensajes en formato: ID#DATA (por ejemplo: 123#11.22.33).
 * Internamente, se construye y ejecuta el comando `cansend` sobre la interfaz especificada.
 * Es util para pruebas iniciales o envío manual de comandos.
 */

// =======================
//        INCLUDES
// =======================

#include <stdio.h>     // Entrada/salida estandar
#include <stdlib.h>    // Funciones como system()
#include <string.h>    // Manipulacion de cadenas (strcmp, strcspn)

// =======================
//       CONSTANTES
// =======================

#define CAN_INTERFACE "can0"  // Interfaz CAN por defecto (puede modificarse segun configuracion)

// =========================
//     FUNCION PRINCIPAL
// =========================

int main() {
    char input[100];   // Buffer para entrada del usuario

    printf("CAN Sender iniciado. Introduce un mensaje en formato: COB_ID#DATA\n");

    while (1) {
        // Solicitar entrada al usuario
        printf("\nIntroduce un mensaje CAN (o escribe 'exit' para salir): ");
        fgets(input, sizeof(input), stdin);  // Leer linea de entrada

        // Eliminar el salto de linea final (\n)
        input[strcspn(input, "\n")] = 0;

        // Comprobar si el usuario desea salir
        if (strcmp(input, "exit") == 0) {
            break;
        }

        // Construir el comando cansend completo
        char command[150];
        snprintf(command, sizeof(command), "cansend %s %s", CAN_INTERFACE, input);

        // Ejecutar el comando y mostrar resultado
        int ret = system(command);
        if (ret != 0) {
            printf("Error\n");  // Error al ejecutar el comando
        } else {
            printf("Mensaje enviado: %s\n", input);  // Confirmacion de envio
        }
    }

    return 0;  // Fin del programa
}
