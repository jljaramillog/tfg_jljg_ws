/**
 * Programa para escuchar mensajes en el bus CAN usando 'candump', con opcion de multiples filtros por COB-ID.
 *
 * El usuario puede introducir hasta 5 COB-IDs separados por espacios (ejemplo: 080 701 181).
 * Si no se introduce nada, se muestran todos los mensajes del bus CAN.
 */

// =======================
//        INCLUDES
// =======================

#include <stdio.h>     // Entrada/salida estandar
#include <stdlib.h>    // Funcion system()
#include <string.h>    // Manipulacion de cadenas

// =======================
//       CONSTANTES
// =======================

#define CAN_INTERFACE "can0"   // Nombre de la interfaz CAN
#define MAX_FILTROS 5          // Numero maximo de COB-IDs que se pueden introducir
#define BUFFER_TAM 100         // Tamano del buffer de entrada

// =========================
//     FUNCION PRINCIPAL
// =========================

int main() {
    char entrada[BUFFER_TAM];    // Linea de entrada del usuario
    char command[256];           // Comando final que se ejecutara con system()
    char filtros[128] = "";      // Parte del comando con los filtros CAN

    printf("Introduce hasta %d COB-IDs separados por espacio (ejemplo: 080 701 181), o ENTER para sin filtro:\n", MAX_FILTROS);
    fgets(entrada, sizeof(entrada), stdin);  // Leer linea completa

    // Eliminar salto de linea final
    entrada[strcspn(entrada, "\n")] = 0;

    // Comprobar si se ha introducido algun filtro
    if (strlen(entrada) > 0) {
        int count = 0;
        char *token = strtok(entrada, " ");
        while (token != NULL && count < MAX_FILTROS) {
            char filtro_str[20];
            snprintf(filtro_str, sizeof(filtro_str), "%s,%s:7FF ", CAN_INTERFACE, token);
            strcat(filtros, filtro_str);
            token = strtok(NULL, " ");
            count++;
        }

        // Construir comando final con filtros multiples
        snprintf(command, sizeof(command), "candump %s", filtros);
        printf("Aplicando filtros a los siguientes COB-IDs: %s\n", filtros);
    } else {
        // Sin filtro
        snprintf(command, sizeof(command), "candump %s", CAN_INTERFACE);
        printf("Mostrando todos los mensajes CAN.\n");
    }

    // Ejecutar el comando resultante
    system(command);

    return 0;
}
