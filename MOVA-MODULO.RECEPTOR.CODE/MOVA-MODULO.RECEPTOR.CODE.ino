/*
 * ============================================================================
 *  MÓDULO RECEPTOR - MOUSE GIROSCÓPICO INALÁMBRICO
 * ============================================================================
 *  Arduino Pro Micro (ATmega32U4) -> conectado por USB al PC
 *  NRF24L01 (recibe los paquetes del emisor)
 *  HID Mouse (mueve el cursor en Windows / Linux / macOS)
 *
 *  Este receptor está alineado con el emisor "MOVA-MODULO.EMISOR.CODE.ino":
 *  misma estructura de datos, misma dirección, mismo canal, mismo data rate.
 *
 *  Qué hace:
 *    1. Recibe por NRF24L01 la estructura MouseData enviada por el emisor.
 *    2. Verifica que la comunicación sea estable (timeout de señal).
 *    3. Convierte moveX/moveY en movimiento HID con Mouse.move().
 *    4. Gestiona los clics por detección de flanco (press/release | presionar/liberar).
 *    5. Si pasan más de 200 ms sin datos: detiene el cursor y suelta botones.
 *    6. Informa por Serial: inicio, NRF detectado, paquetes, pérdida, reconexión.
 *
 * CONEXIONES — Pro Micro <-> NRF24L01:
 * ┌─────────────────┬──────────────┬──────────────────────────┐
 * │  Pro Micro Pin  │  NRF24L01    │  Descripción             │
 * ├─────────────────┼──────────────┼──────────────────────────┤
 * │  3.3V  (¡no 5V!)│  VCC         │  Alimentación 3.3V       │
 * │  GND            │  GND         │  Tierra común            │
 * │  Pin 9          │  CE          │  Chip Enable             │
 * │  Pin 10         │  CSN         │  Chip Select             │
 * │  Pin 15 (SCK)   │  SCK         │  Reloj SPI               │
 * │  Pin 16 (MOSI)  │  MOSI        │  Datos SPI salida        │
 * │  Pin 14 (MISO)  │  MISO        │  Datos SPI entrada       │
 * │  (sin conectar) │  IRQ         │  No se usa               │
 * └─────────────────┴──────────────┴──────────────────────────┘
 *  NOTA: capacitor de 47µF entre VCC y GND junto al módulo NRF24.
 * 
 *  COMPILACIÓN:
 *   - Board: "SparkFun Pro Micro" o "Arduino Leonardo"
 * 
 * ============================================================================
*/

#include <SPI.h>        // Bus SPI para el NRF24L01            (incluida en Arduino IDE)
#include <nRF24L01.h>   // Definiciones de registros del NRF24  (librería RF24)
#include <RF24.h>       // Control de alto nivel del NRF24L01   (librería RF24, TMRh20)
#include <Mouse.h>      // Emulación de mouse USB HID           (incluida en el core ATmega32U4)


struct MouseData {
  int16_t moveX;       // Movimiento horizontal (+ derecha / - izquierda)
  int16_t moveY;       // Movimiento vertical   (+ abajo   / - arriba)
  bool clickLeft;      // Estado del botón izquierdo
  bool clickRight;     // Estado del botón derecho
  uint8_t magic;       // Valor centinela: el receptor solo acepta paquetes cuyo 'magic' coincida con el del emisor
};

MouseData data; // Aquí se vuelca cada paquete recibido

#define MAGIC_VALOR 0x5A

/* ============================================================================
 *  CONFIGURACIÓN DEL NRF24L01 (debe coincidir EXACTAMENTE con el emisor)
 * ============================================================================
*/
#define PIN_CE   9     // Chip Enable del NRF24L01
#define PIN_CSN  10    // Chip Select del NRF24L01
// SPI por hardware del Pro Micro: MOSI=16, MISO=14, SCK=15
RF24 radio(PIN_CE, PIN_CSN);

// Dirección lógica del pipe (5 bytes). Igual a la del emisor.
const byte direccion[6] = "MOUSE";

// LED indicador de error (pin 17 = LED RX del Pro Micro; enciende en LOW).
#define PIN_LED_ERROR  17

/* ============================================================================
 *  PARÁMETROS AJUSTABLES
 * ============================================================================
*/

// Tiempo máximo sin recibir paquetes antes de declarar "señal perdida" (ms).
const unsigned long TIMEOUT_MS = 200;

// Cada cuánto se imprime el resumen de paquetes recibidos por Serial (ms).
const unsigned long DEBUG_MS = 1000;

// Cada cuánto se "re-arma" la radio MIENTRAS no hay señal, para recuperarse
// de un estado bloqueado del módulo y garantizar la reconexión automática.
const unsigned long RESCAN_MS = 250;

// Límite de seguridad por eje: Mouse.move() admite valores de -127 a 127.
const int8_t MOVE_CLAMP = 127;

/* ============================================================================
 *  VARIABLES INTERNAS DE ESTADO
 * ============================================================================
*/

unsigned long ultimoPaquete = 0;   // millis() del último paquete válido
unsigned long ultimoDebug   = 0;   // millis() del último resumen por Serial
unsigned long ultimoRescan  = 0;   // millis() del último re-armado de la radio
unsigned long contadorPaq   = 0;   // Paquetes recibidos desde el último resumen

bool conectado   = false;          // ¿Estamos recibiendo señal ahora mismo?
bool huboPerdida = false;          // ¿Hubo una pérdida previa? (para distinguir reconexión)

// Estado anterior de los botones (para detectar flancos press/release).
bool prevLeft  = false;
bool prevRight = false;

/* ============================================================================
 *  FUNCIONES AUXILIARES
 * ============================================================================
*/

// Suelta cualquier botón que pudiera haber quedado presionado y resetea estados.
// Se usa al perder la señal para evitar clics "pegados".
void liberarBotones() {
  if (prevLeft)  Mouse.release(MOUSE_LEFT);
  if (prevRight) Mouse.release(MOUSE_RIGHT);
  prevLeft  = false;
  prevRight = false;
}

// Parpadeo NO bloqueante del LED de error (sin delay()), mientras el NRF24L01
// no responde. Reintenta begin() para recuperarse si lo reconectas.
void errorNRF() {
  unsigned long tLed = 0;
  bool estadoLed = false;
  Serial.println(F("ERROR: NRF24L01 no detectado. Revisa cableado y 3.3V."));

  while (!radio.begin()) {                 // Sale cuando el NRF responde
    unsigned long ahora = millis();
    if (ahora - tLed >= 250) {
      tLed = ahora;
      estadoLed = !estadoLed;
      digitalWrite(PIN_LED_ERROR, estadoLed ? LOW : HIGH); // LOW = encendido
    }
  }
  digitalWrite(PIN_LED_ERROR, HIGH);       // Apaga el LED (NRF OK)
}


void setup() {
  
  pinMode(PIN_LED_ERROR, OUTPUT);
  digitalWrite(PIN_LED_ERROR, HIGH);       // LED apagado al inicio

  Serial.begin(115200);

  unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { /* espera breve */ };
  Serial.println(F("== Receptor de mouse giroscopico =="));
  Serial.println(F("Inicio correcto."));

  Mouse.begin();                           // Inicia la emulación de mouse USB

  if (!radio.begin()) {                    // Verifica que el módulo responda
    errorNRF();                            // Bloquea con parpadeo hasta detectarlo
  }
  Serial.println(F("NRF24L01 detectado."));

  // DIAGNÓSTICO SPI
  Serial.print(F("Chip NRF conectado por SPI: "));
  Serial.println(radio.isChipConnected() ? F("SI") : F("NO"));

  // Configuración (idéntica al emisor):
  radio.setChannel(108);                   // Canal RF fijo (0-125)
  radio.setDataRate(RF24_1MBPS);           // 1 Mbps: baja latencia y buen alcance
  radio.setPALevel(RF24_PA_MIN);           // Potencia mínima.
  radio.setAutoAck(true);                  // Confirmación automática de paquetes
  radio.openReadingPipe(1, direccion);     // Escucha en el pipe 1 con la dirección común.
  radio.startListening();                  // Modo RECEPTOR.

  Serial.println(F("Escuchando paquetes..."));
  ultimoPaquete = millis();                // Evita un timeout inmediato al arrancar
}


void loop() {
  unsigned long ahora = millis();

  /* ------------------------------------------------------------------------
   *  (1) RECEPCIÓN DE PAQUETES
   * ------------------------------------------------------------------------ 
  */

  if (radio.available()) {
    radio.read(&data, sizeof(data));       // Vuelca el paquete en la struct

    if (data.magic != MAGIC_VALOR) {
      radio.flush_rx();                    // Limpia datos inválidos del FIFO
    } else {
      ultimoPaquete = ahora;               // Paquete REAL: reinicia el timeout
      contadorPaq++;

      // Transición a "conectado".
      if (!conectado) {
        conectado = true;
        if (huboPerdida) Serial.println(F(">> Reconexion: senal recuperada."));
        else             Serial.println(F(">> Senal establecida: recibiendo datos."));
      }

      /* --------------------------------------------------------------------
       *  2) MOVIMIENTO DEL CURSOR (HID)
       *  Los valores ya vienen filtrados y limitados desde el emisor.
       * -------------------------------------------------------------------- 
      */

      int8_t mx = (int8_t)constrain(data.moveX, -MOVE_CLAMP, MOVE_CLAMP);
      int8_t my = (int8_t)constrain(data.moveY, -MOVE_CLAMP, MOVE_CLAMP);
      if (mx != 0 || my != 0) {
        Mouse.move(mx, my, 0);             // Movimiento relativo (x, y, rueda)
      }

      /* --------------------------------------------------------------------
       *  3) CLIC IZQUIERDO (detección de flanco)
       * --------------------------------------------------------------------
      */

      if (data.clickLeft && !prevLeft) {
        Mouse.press(MOUSE_LEFT);           // Flanco de subida -> presiona
      } else if (!data.clickLeft && prevLeft) {
        Mouse.release(MOUSE_LEFT);         // Flanco de bajada -> suelta
      }
      prevLeft = data.clickLeft;

      /* --------------------------------------------------------------------
       *  4) CLIC DERECHO (detección de flanco)
       * --------------------------------------------------------------------
      */

      if (data.clickRight && !prevRight) {
        Mouse.press(MOUSE_RIGHT);
      } else if (!data.clickRight && prevRight) {
        Mouse.release(MOUSE_RIGHT);
      }
      prevRight = data.clickRight;
    }
  }

  /* ------------------------------------------------------------------------
   *  5) PÉRDIDA DE SEÑAL
   *  Si pasan más de TIMEOUT_MS sin paquetes: el cursor ya se detiene solo
   *  (no llamamos a Mouse.move sin datos) y soltamos los botones para que no
   *  queden "pegados". El receptor NO se rinde: sigue escuchando y, además,
   *  re-arma la radio periódicamente (paso 6) hasta que el emisor vuelva.
   * ------------------------------------------------------------------------
  */

  if (conectado && (ahora - ultimoPaquete > TIMEOUT_MS)) {
    conectado   = false;
    huboPerdida = true;
    liberarBotones(); // Evita clics atascados
    Serial.println(F(">> Senal perdida: cursor detenido. Buscando emisor..."));
  }


  /* ------------------------------------------------------------------------
   *  6) RE-ARMADO DE LA RADIO MIENTRAS NO HAY SEÑAL
   *  Algunos módulos NRF24 (sobre todo clones) quedan en un estado bloqueado
   *  tras una caída y dejan de recibir aunque el emisor vuelva a transmitir.
   *  Vaciar el FIFO de recepción y re-activar la escucha cada RESCAN_MS los
   *  saca de ese estado y garantiza la reconexión automática. Es seguro
   *  hacerlo aquí porque, al no haber señal, no hay paquetes que perder.
   * ------------------------------------------------------------------------
  */

  if (!conectado && (ahora - ultimoRescan >= RESCAN_MS)) {
    ultimoRescan = ahora;
    radio.flush_rx();                      // Descarta datos viejos/corruptos
    radio.startListening();                // Re-asegura el modo escucha
  }

  /* ------------------------------------------------------------------------
   *  7) RESUMEN DE DEPURACIÓN (Serial)
   *  Imprime cada DEBUG_MS el número de paquetes recibidos en el último segundo.
   * ------------------------------------------------------------------------
  */

  if (conectado && (ahora - ultimoDebug >= DEBUG_MS)) {
    ultimoDebug = ahora;
    Serial.print(F("Paquetes recibidos (~ultimo s): "));
    Serial.println(contadorPaq);
    contadorPaq = 0;
  }
}
