#include "LobotServoController.h"

#define OBTENER_BYTE_BAJO(A) (uint8_t)((A))
//Macro para obtener los 8 bits bajos de A
#define OBTENER_BYTE_ALTO(A) (uint8_t)((A) >> 8)
//Macro para obtener los 8 bits altos de A
#define BYTE_A_HW(A, B) ((((uint16_t)(A)) << 8) | (uint8_t)(B))
//Macro para combinar A como byte alto y B como byte bajo en un entero de 16 bits

LobotServoController::LobotServoController()
{
    // Inicializar el número del grupo de acción en ejecución como 0xFF, número de ejecuciones como 0, estado de ejecución como falso, y voltaje de la batería como 0
    numOfActinGroupRunning = 0xFF;
    actionGroupRunTimes = 0;
    isRunning = false;
    batteryVoltage = 0;
#if defined(__AVR_ATmega32U4__)  //para Arduino Leonardo, Micro....
    SerialX = &Serial1;
#else
    SerialX = &Serial;
#endif
}

LobotServoController::LobotServoController(HardwareSerial &A)
{
    LobotServoController();
    SerialX = &A;
}
LobotServoController::~LobotServoController()
{
}

/*********************************************************************************
 * Función:  moverServo
 * Descripción: Controla el movimiento de un solo servo
 * Parámetros:  servoID: ID del servo, Position: posición objetivo, Time: tiempo de movimiento
 *              ID del servo: 0<= servoID <=31, Time: Time > 0
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::moverServo(uint8_t servoID, uint16_t Position, uint16_t Time)
{
    uint8_t buf[11];
    if (servoID > 31 || !(Time > 0)) { // El ID del servo no puede ser mayor a 31
        return;
    }
    buf[0] = CABECERA_FRAME;             // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = 8;                          // Longitud de datos = número de servos * 3 + 5, en este caso = 1 * 3 + 5
    buf[3] = CMD_SERVO_MOVER;            // Llenar comando de movimiento del servo
    buf[4] = 1;                          // Número de servos a controlar
    buf[5] = OBTENER_BYTE_BAJO(Time);    // Llenar los 8 bits bajos del tiempo
    buf[6] = OBTENER_BYTE_ALTO(Time);    // Llenar los 8 bits altos del tiempo
    buf[7] = servoID;                    // ID del servo
    buf[8] = OBTENER_BYTE_BAJO(Position);// Llenar los 8 bits bajos de la posición objetivo
    buf[9] = OBTENER_BYTE_ALTO(Position);// Llenar los 8 bits altos de la posición objetivo

    SerialX->write(buf, 10);
}

/*********************************************************************************
 * Función:  moverServos
 * Descripción: Controla el movimiento de múltiples servos
 * Parámetros:  servos[]: arreglo de estructuras de servos, Num: número de servos, Time: tiempo de movimiento
 *              0 < Num <= 32, Time > 0
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::moverServos(LobotServo servos[], uint8_t Num, uint16_t Time)
{
    uint8_t buf[103];    // Crear buffer
    if (Num < 1 || Num > 32 || !(Time > 0)) {
        return; // El número de servos no puede ser cero ni mayor a 32, el tiempo no puede ser cero
    }
    buf[0] = CABECERA_FRAME;    // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = Num * 3 + 5;       // Longitud de datos = número de servos * 3 + 5
    buf[3] = CMD_SERVO_MOVER;   // Llenar comando de movimiento del servo
    buf[4] = Num;               // Número de servos a controlar
    buf[5] = OBTENER_BYTE_BAJO(Time); // Obtener los 8 bits bajos del tiempo
    buf[6] = OBTENER_BYTE_ALTO(Time); // Obtener los 8 bits altos del tiempo
    uint8_t index = 7;
    for (uint8_t i = 0; i < Num; i++) { // Llenar en bucle los IDs de los servos y sus posiciones objetivo
        buf[index++] = servos[i].ID; // Llenar ID del servo
        buf[index++] = OBTENER_BYTE_BAJO(servos[i].Position); // Llenar los 8 bits bajos de la posición objetivo
        buf[index++] = OBTENER_BYTE_ALTO(servos[i].Position);// Llenar los 8 bits altos de la posición objetivo
    }
    SerialX->write(buf, buf[2] + 2); // Enviar frame, longitud = longitud de datos + dos bytes de cabecera
}

/*********************************************************************************
 * Función:  moverServos
 * Descripción: Controla el movimiento de múltiples servos
 * Parámetros:  Num: número de servos, Time: tiempo de movimiento,...: ID del servo, ángulo de movimiento, ID del servo, ángulo de movimiento, y así sucesivamente
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::moverServos(uint8_t Num, uint16_t Time, ...)
{
    uint8_t buf[128];
    va_list arg_ptr = NULL;
    va_start(arg_ptr, Time); // Obtener la dirección del primer parámetro variable
    if (Num < 1 || Num > 32 || (!(Time > 0)) || arg_ptr == NULL) {
        return; // El número de servos no puede ser cero ni mayor a 32, el tiempo no puede ser cero, y los parámetros variables no pueden estar vacíos
    }
    buf[0] = CABECERA_FRAME;     // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = Num * 3 + 5;        // Longitud de datos = número de servos * 3 + 5
    buf[3] = CMD_SERVO_MOVER;    // Comando de movimiento del servo
    buf[4] = Num;                // Número de servos a controlar
    buf[5] = OBTENER_BYTE_BAJO(Time); // Obtener los 8 bits bajos del tiempo
    buf[6] = OBTENER_BYTE_ALTO(Time); // Obtener los 8 bits altos del tiempo
    uint8_t index = 7;
    for (uint8_t i = 0; i < Num; i++) { // Obtener en bucle los IDs de los servos y sus posiciones objetivo de los parámetros variables
        uint16_t tmp = va_arg(arg_ptr, uint16_t); // Obtener ID del servo de los parámetros variables
        buf[index++] = OBTENER_BYTE_BAJO(tmp); // Obtener los 8 bits bajos
        uint16_t pos = va_arg(arg_ptr, uint16_t); // Obtener posición objetivo de los parámetros variables
        buf[index++] = OBTENER_BYTE_BAJO(pos); // Llenar los 8 bits bajos de la posición objetivo
        buf[index++] = OBTENER_BYTE_ALTO(pos); // Llenar los 8 bits altos de la posición objetivo
    }
    va_end(arg_ptr);     // Vaciar arg_ptr
    SerialX->write(buf, buf[2] + 2); // Enviar frame
}

/*********************************************************************************
 * Función:  ejecutarGrupoAccion
 * Descripción: Ejecuta el grupo de acciones especificado
 * Parámetros:  numOfAction: número del grupo de acción, Times: número de veces a ejecutar
 * Retorno:     No retorna
 * Otros:       Times = 0 para bucle infinito
 **********************************************************************************/
void LobotServoController::ejecutarGrupoAccion(uint8_t numOfAction, uint16_t Times)
{
    uint8_t buf[7];
    buf[0] = CABECERA_FRAME;   // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = 5;      // Longitud de datos, longitud del frame de datos excluyendo la cabecera, este comando es fijo a 5
    buf[3] = CMD_GRUPO_ACCION_EJECUTAR; // Llenar comando de ejecutar grupo de acciones
    buf[4] = numOfAction;      // Llenar número del grupo de acción a ejecutar
    buf[5] = OBTENER_BYTE_BAJO(Times); // Obtener los 8 bits bajos del número de veces a ejecutar
    buf[6] = OBTENER_BYTE_ALTO(Times); // Obtener los 8 bits altos del número de veces a ejecutar
    SerialX->write(buf, 7);      // Enviar frame de datos
}

/*********************************************************************************
 * Función:  detenerGrupoAccion
 * Descripción: Detiene la ejecución del grupo de acciones
 * Parámetros:  Ninguno
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::detenerGrupoAccion(void)
{
    uint8_t buf[4];
    buf[0] = CABECERA_FRAME;     // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = 2;                  // Longitud de datos, longitud del frame de datos excluyendo la cabecera, este comando es fijo a 2
    buf[3] = CMD_GRUPO_ACCION_DETENER; // Llenar comando de detener grupo de acciones
    SerialX->write(buf, 4);      // Enviar frame de datos
}

/*********************************************************************************
 * Función:  establecerVelocidadGrupoAccion
 * Descripción: Establece la velocidad de ejecución del grupo de acciones especificado
 * Parámetros:  numOfAction: número del grupo de acción, Speed: velocidad objetivo
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::establecerVelocidadGrupoAccion(uint8_t numOfAction, uint16_t Speed)
{
    uint8_t buf[7];
    buf[0] = CABECERA_FRAME;     // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = 5;                  // Longitud de datos, longitud del frame de datos excluyendo la cabecera, este comando es fijo a 5
    buf[3] = CMD_GRUPO_ACCION_VELOCIDAD; // Llenar comando de establecer velocidad del grupo de acciones
    buf[4] = numOfAction;        // Llenar número del grupo de acción a establecer
    buf[5] = OBTENER_BYTE_BAJO(Speed); // Obtener los 8 bits bajos de la velocidad objetivo
    buf[6] = OBTENER_BYTE_ALTO(Speed); // Obtener los 8 bits altos de la velocidad objetivo
    SerialX->write(buf, 7);      // Enviar frame de datos
}

/*********************************************************************************
 * Función:  establecerVelocidadTodosGruposAccion
 * Descripción: Establece la velocidad de ejecución para todos los grupos de acciones
 * Parámetros:  Speed: velocidad objetivo
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::establecerVelocidadTodosGruposAccion(uint16_t Speed)
{
    establecerVelocidadGrupoAccion(0xFF, Speed); // Llamar a la función de establecer velocidad del grupo de acciones, cuando el número del grupo es 0xFF, establece la velocidad de todos los grupos
}

/*********************************************************************************
 * Función:  obtenerVoltajeBateria
 * Descripción: Envía comando para obtener el voltaje de la batería
 * Parámetros:  Ninguno
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::obtenerVoltajeBateria()
{
    uint8_t buf[4];
    buf[0] = CABECERA_FRAME;         // Llenar cabecera de frame
    buf[1] = CABECERA_FRAME;
    buf[2] = 2;                     // Longitud de datos, longitud del frame de datos excluyendo la cabecera, este comando es fijo a 2
    buf[3] = CMD_OBTENER_VOLTAGE_BATERIA; // Llenar comando para obtener voltaje de la batería
    SerialX->write(buf, 4);        // Enviar frame de datos
}

/*********************************************************************************
 * Función:  manejarRecepcion
 * Descripción: Maneja la recepción de datos en el puerto serial
 * Parámetros:  Ninguno
 * Retorno:     No retorna
 **********************************************************************************/
void LobotServoController::manejarRecepcion()
{
    uint8_t buf[16];
    static uint8_t len = 0;
    static uint8_t getHeader = 0;
    if (!SerialX->available())
        return;
    // Si no hay datos, retornar
    do {
        switch (getHeader) {
        case 0:
            if (SerialX->read() == CABECERA_FRAME)
                getHeader = 1;
            break;
        case 1:
            if (SerialX->read() == CABECERA_FRAME)
                getHeader = 2;
            else
                getHeader = 0;
            break;
        case 2:
            len = SerialX->read();
            getHeader = 3;
            break;
        case 3:
            if (SerialX->readBytes(buf, len - 1) > 0)
                getHeader = 4;
            else {
                len = 0;
                getHeader = 0;
                break;
            }
        case 4:
            switch (buf[0]) {
            case VOLTAJE_BATERIA: // Comando de voltaje de la batería
                batteryVoltage = BYTE_A_HW(buf[2], buf[1]); // Combinar los 8 bits altos y bajos para obtener el voltaje de la batería
                break;
            case GRUPO_ACCION_EJECUTANDO: // Un grupo de acciones está en ejecución
                numOfActinGroupRunning = buf[1]; // Obtener el número del grupo de acción en ejecución
                actionGroupRunTimes = BYTE_A_HW(buf[3], buf[2]); // Combinar los 8 bits altos y bajos para obtener el número de ejecuciones
                isRunning = true; // Establecer estado de ejecución a verdadero
                break;
            case GRUPO_ACCION_DETENIDO: // Grupo de acciones detenido
            case  GRUPO_ACCION_COMPLETADO:// Grupo de acciones completado
                isRunning = false; // Establecer estado de ejecución a falso
                numOfActinGroupRunning = 0xFF; // Establecer el número del grupo de acción en ejecución a 0xFF
                actionGroupRunTimes = 0; // Establecer el número de ejecuciones a 0
                break;
            default:
                break;
            }
        default:
            len = 0;
            getHeader = 0;
            break;
        }
    } while (SerialX->available());
}


