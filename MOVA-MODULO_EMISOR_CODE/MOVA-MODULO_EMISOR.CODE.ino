/*
 * ============================================================================
 *  MÓDULO EMISOR — HEAD TRACKER INALÁMBRICO
 *  Arduino Pro Micro (ATmega32U4) + MPU6050 (GY-521) + NRF24L01
 * ============================================================================
 *
 * QUÉ HACE:
 *   Lee el acelerómetro y el giroscopio del MPU6050 mediante la librería
 *   MPU6050.h, calcula el movimiento de cabeza (horizontal/vertical) con un
 *   filtro complementario, le aplica zona muerta, sensibilidad, suavizado y
 *   límite, y TRANSMITE el resultado por NRF24L01 al módulo receptor (que es
 *   quien actúa como mouse USB en el PC).
 *
 * CONEXIONES — Pro Micro <-> MPU6050 (GY-521):
 * ┌─────────────────┬──────────────┬──────────────────────────┐
 * │  Pro Micro Pin  │  GY-521 Pin  │  Descripción             │
 * ├─────────────────┼──────────────┼──────────────────────────┤
 * │  5V             │  VCC         │  Alimentación (5V o 3.3V)│
 * │  GND            │  GND         │  Tierra común            │
 * │  Pin 2 (SDA)    │  SDA         │  I2C Datos               │
 * │  Pin 3 (SCL)    │  SCL         │  I2C Reloj               │
 * │  (sin conectar) │  AD0         │  Libre → dirección 0x68  │
 * │  (sin conectar) │  INT         │  No se usa               │
 * └─────────────────┴──────────────┴──────────────────────────┘
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
 * COMPILACIÓN:
 *   - Board: "SparkFun Pro Micro" o "Arduino Leonardo"
 * 
 * ============================================================================
*/

#include <Wire.h>       // I2C: bus de comunicación con el MPU6050   (incluida en Arduino IDE)
#include <SPI.h>        // SPI: bus de comunicación con el NRF24L01  (incluida en Arduino IDE)
#include <nRF24L01.h>   // Definiciones de registros del NRF24L01     (librería RF24)
#include <RF24.h>       // Control de alto nivel del NRF24L01         (librería RF24, TMRh20)
#include <MPU6050.h>    // Driver del MPU6050 (lectura accel/gyro)    (librería MPU6050, Electronic Cats)

#include <printf.h>


struct MouseData {
  int16_t moveX;       // Movimiento horizontal a enviar
  int16_t moveY;       // Movimiento vertical a enviar
  bool    clickLeft;   // Click izquierdo (pin PIN_BTN_LEFT,  INPUT_PULLUP)
  bool    clickRight;  // Click derecho   (pin PIN_BTN_RIGHT, INPUT_PULLUP)
  uint8_t magic;       // Valor centinela: el receptor solo acepta paquetes cuyo 'magic' coincida con el del emisor
};

MouseData data;        // Instancia que se envía en cada ciclo

// Valor centinela: el receptor solo acepta paquetes cuyo 'magic' coincida.
#define MAGIC_VALOR 0x5A

/* ============================================================
 * OBJETOS DE HARDWARE
 * ============================================================
*/

MPU6050 mpu;           // Sensor MPU6050 (dirección I2C 0x68 por defecto)

#define PIN_CE   9     // Chip Enable del NRF24L01
#define PIN_CSN  10    // Chip Select del NRF24L01
// SPI por hardware del Pro Micro: MOSI=16, MISO=14, SCK=15
RF24 radio(PIN_CE, PIN_CSN);

// --- LED RGB (ánodo común: LOW = encendido, HIGH = apagado) ---
#define PIN_LED_R  A0   // Rojo
#define PIN_LED_B  A1   // Azul
#define PIN_LED_G  A2   // Verde

// --- Pines de los botones del mouse ---
#define PIN_BTN_LEFT  4   // Botón izquierdo (INPUT_PULLUP: LOW = presionado)
#define PIN_BTN_RIGHT 5   // Botón derecho   (INPUT_PULLUP: LOW = presionado)

// Dirección lógica del pipe (5 bytes). Debe ser igual a la del receptor.
const byte direccion[6] = "MOUSE";

// ============================================================
//  PARÁMETROS AJUSTABLES
// ============================================================

// --- Sensibilidad del movimiento (por eje) ---
const float SENSITIVITY_X = 1.0;  // Horizontal
const float SENSITIVITY_Y = 1.0;  // Vertical

// --- Zona muerta (deadzone) en grados ---
const float DEADZONE = 3.0;

// --- Filtro complementario (fusión accel + gyro) valor de 0.00 a 1.00 ---
// Cerca de 1.00 confía más en el giroscopio (suave, con algo de drift);
// cerca de 0.00 confía más en el acelerómetro (menos drift, más ruido).
const float ALPHA = 0.96;

// --- Suavizado (media móvil exponencial) valor de 0.00 a 1.00 ---
// Cerca de 1.0 = muy suave pero con retardo
// Cerca de 0.0 = rápido pero brusco.
const float SMOOTHING = 0.70;

// --- Límite máximo de movimiento por ciclo ---
const int MAX_CURSOR_SPEED = 20;

// --- Intervalo de muestreo y transmisión (ms) ---
const int LOOP_INTERVAL_MS = 10;

// --- Número de muestras para la calibración inicial del girosccopio ---
const int CALIBRATION_SAMPLES = 100;

// ============================================================
//  ESCALAS DEL MPU6050
//  La librería, tras initialize(), deja el sensor en ±2g y ±250°/s
//  cuyas escalas son éstas:
// ============================================================
const float ACCEL_SCALE = 16384.0; // ±2g    → 16384 LSB/g
const float GYRO_SCALE  = 131.0;   // ±250°/s→ 131 LSB/(°/s)

// ============================================================
//  VARIABLES GLOBALES
// ============================================================
float gyroOffsetX = 0.0, gyroOffsetY = 0.0, gyroOffsetZ = 0.0; // Offsets gyro
float angleX = 0.0, angleY = 0.0;       // Ángulos del filtro complementario
float refAngleX = 0.0, refAngleY = 0.0; // Posición neutra de la cabeza
float smoothMouseX = 0.0, smoothMouseY = 0.0; // Movimiento suavizado

unsigned long lastTime = 0;             // Control de frecuencia del loop

unsigned long lastSerialPrint = 0;      // Control del debug por Serial
const int SERIAL_INTERVAL_MS = 1000;     // Imprimir cada 100 ms

int countOK   = 0;
int countFAIL = 0;


void esperar(unsigned long ms) {
  unsigned long t = millis();
  while (millis() - t < ms) { /* espera activa */ }
}

void LedRGB(bool rojo, bool azul, bool verde) {
  digitalWrite(PIN_LED_R, rojo  ? HIGH : LOW);
  digitalWrite(PIN_LED_B, azul  ? HIGH : LOW);
  digitalWrite(PIN_LED_G, verde ? HIGH : LOW);
}

// Colores de estado (macros para legibilidad)
#define LED_APAGADO()   LedRGB(0, 0, 0)
#define LED_AMARILLO()  LedRGB(1, 0, 1)   // Rojo + Verde = Amarillo (iniciando)
#define LED_AZUL()      LedRGB(0, 1, 0)   // Calibrando
#define LED_VERDE()     LedRGB(0, 0, 1)   // OK
#define LED_ROJO()      LedRGB(1, 0, 0)   // Error hardware
#define LED_MAGENTA()   LedRGB(1, 1, 0) 

void setup() {
  
  Serial.begin(115200);
  printf_begin();
  unsigned long startWait = millis();

  // --- Inicialización pines LED RGB ---
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  LED_AMARILLO();   // Amarillo: iniciando / esperando Serial

  while (!Serial && (millis() - startWait < 3000)) { /* espera breve */ }

  Serial.println(F("============================================"));
  Serial.println(F("  EMISOR HEAD TRACKER (MPU6050 lib + NRF24)"));
  Serial.println(F("============================================"));

  // Verificación final: ambos periféricos deben estar OK
  bool mpuOK = false;
  bool rfOK  = false;

  // --- Pines de botones ---
  pinMode(PIN_BTN_LEFT,  INPUT_PULLUP);
  pinMode(PIN_BTN_RIGHT, INPUT_PULLUP);

  // --- Inicialización Bus I2C ---
  Wire.begin();
  Wire.setClock(400000); //400kHz

  mpu.initialize();
  mpu.setDLPFMode(MPU6050_DLPF_BW_42);

  uint8_t id = mpu.getDeviceID();
  Serial.print("WHO_AM_I = 0x");
  Serial.println(id, HEX);

  if (mpu.testConnection()) {
    Serial.println(F("[MPU6050] Conexion correcta."));
    mpuOK = true;
  } else {
    Serial.println(F("[MPU6050] ERROR: no responde. Revisar I2C (SDA, SCL) y alimentacion."));
    mpuOK = false;
  }

  Serial.println(F("[CALIBRACION] Mantener cabeza QUIETA mirando al frente..."));
  Serial.println(F("[CALIBRACION] Iniciando en 2 segundos..."));
  LED_AZUL();
  esperar(2000);
  calibrarMPU6050();
  Serial.println(F("[CALIBRACION] Completada.\n"));

  // --- Inicialización NRF24L01 ---
  radio.begin();

  // Configuración (idéntica al receptor):
  radio.setChannel(108);            // Canal RF fijo (0-125). Igual en el receptor.
  radio.setDataRate(RF24_1MBPS);    // 1 Mbps: baja latencia y buen alcance.
  radio.setPALevel(RF24_PA_MIN);    // Potencia mínima.
  radio.setAutoAck(true);           // Confirmación automática de paquetes.
  radio.openWritingPipe(direccion); // Canal de escritura.
  radio.stopListening();            // Modo TRANSMISOR.

  Serial.print(F("[NRF24L01] Chip conectado por SPI: "));
  if(radio.isChipConnected()){
    Serial.println(F("SI"));
    rfOK = true;
  }
  else{
    Serial.println(F("NO"));
    rfOK = false;
  }

  if (!mpuOK || !rfOK) {
    if (!mpuOK) Serial.println(F("[ERROR] MPU6050 no detectado."));
    if (!rfOK)  Serial.println(F("[ERROR] NRF24L01 no detectado."));
    LED_ROJO();     // Rojo: algún periférico no responde
    while (true){}

  } else {
    LED_VERDE();    // Verde: todo OK, listo para transmitir
  }

  // Estado inicial de los botones (se leen en tiempo real en el loop).
  data.clickLeft  = false;
  data.clickRight = false;

  Serial.println(F("[OK] Emisor listo. Transmitiendo movimiento...\n"));
  Serial.println(F("gyroX\tgyroY\tangleX\tangleY\tmoveX\tmoveY\tTX"));

  lastTime = millis();
  lastSerialPrint = millis();

  radio.printDetails();
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {

  unsigned long currentTime = millis();

  if ((currentTime - lastTime) < LOOP_INTERVAL_MS) {
    return;
  }

  // Delta de tiempo en segundos (para integrar el giroscopio).
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;
  if (dt > 0.05) dt = 0.05;         // Limita dt si el loop se retrasa.

  /* ------------------------------------------------------------------------
   *  (1) LEER SENSORES con la librería MPU6050
   * ------------------------------------------------------------------------
  */

  int16_t rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;
  mpu.getMotion6(&rawAX, &rawAY, &rawAZ, &rawGX, &rawGY, &rawGZ);

  // Convertir a unidades físicas y restar los offsets del giroscopio.
  float ax = rawAX / ACCEL_SCALE;             // g
  float ay = rawAY / ACCEL_SCALE;             // g
  float az = rawAZ / ACCEL_SCALE;             // g
  float gx = (rawGX / GYRO_SCALE) - gyroOffsetX; // °/s
  float gy = (rawGY / GYRO_SCALE) - gyroOffsetY; // °/s
  // gz no se usa para el movimiento, pero se lee igualmente.

  /* ------------------------------------------------------------------------
   *  (2) FILTRO COMPLEMENTARIO (accel + gyro)
   * ------------------------------------------------------------------------
  */

  float accelAngleX = atan2(ay, az) * 180.0 / PI;  // Roll
  float accelAngleY = atan2(-ax, az) * 180.0 / PI; // Pitch
  angleX = ALPHA * (angleX + gx * dt) + (1.0 - ALPHA) * accelAngleX;
  angleY = ALPHA * (angleY + gy * dt) + (1.0 - ALPHA) * accelAngleY;

  /* ------------------------------------------------------------------------
   *  (3) DESPLAZAMIENTO RELATIVO A LA POSICIÓN NEUTRA
   * ------------------------------------------------------------------------
  */
  
  float deltaX = angleX - refAngleX;  // Roll  → cursor Y
  float deltaY = angleY - refAngleY;  // Pitch → cursor X

  /* ------------------------------------------------------------------------
   *  (4) ZONA MUERTA
   * ------------------------------------------------------------------------
  */
 
  float deadDeltaX = applyDeadzone(deltaX, DEADZONE);
  float deadDeltaY = applyDeadzone(deltaY, DEADZONE);

  /* ------------------------------------------------------------------------
   *  (5) MOVIMIENTO BRUTO (con sensibilidad)
   * ------------------------------------------------------------------------
  */

  float rawMouseX =  deadDeltaY * SENSITIVITY_X;  // Pitch → X
  float rawMouseY = -deadDeltaX * SENSITIVITY_Y;  // Roll  → Y (invertido)

  /* ------------------------------------------------------------------------
   *  (6) SUAVIZADO (media móvil exponencial)
   * ------------------------------------------------------------------------
  */

  smoothMouseX = SMOOTHING * smoothMouseX + (1.0 - SMOOTHING) * rawMouseX;
  smoothMouseY = SMOOTHING * smoothMouseY + (1.0 - SMOOTHING) * rawMouseY;

  /* ------------------------------------------------------------------------
   *  (7) LIMITAR VELOCIDAD MÁXIMA
   * ------------------------------------------------------------------------
  */
 
  int moveX = clampCursorSpeed(smoothMouseX, MAX_CURSOR_SPEED);
  int moveY = clampCursorSpeed(smoothMouseY, MAX_CURSOR_SPEED);

  /* ------------------------------------------------------------------------
   *  (8) TRANSMITIR POR NRF24L01
   * ------------------------------------------------------------------------
  */

  data.moveX      = (int16_t)moveX;
  data.moveY      = (int16_t)moveY;
  data.clickLeft  = digitalRead(PIN_BTN_LEFT);   // LOW = presionado
  data.clickRight = digitalRead(PIN_BTN_RIGHT);  // LOW = presionado
  data.magic      = MAGIC_VALOR;    // Marca el paquete como válido

  bool ok = radio.write(&data, sizeof(data), true);   // multicast = true

  if (ok) countOK++; else countFAIL++;
  //if (countFAIL > 20) LED_AMARILLO(); else LED_VERDE();
  /* ------------------------------------------------------------------------
   *  (9) DEBUG POR SERIAL (cada SERIAL_INTERVAL_MS)
   * ------------------------------------------------------------------------
  */

  if ((currentTime - lastSerialPrint) >= SERIAL_INTERVAL_MS) {
    lastSerialPrint = currentTime;
    Serial.print(gx, 2);     Serial.print(F("\t"));
    Serial.print(gy, 2);     Serial.print(F("\t"));
    Serial.print(angleX, 2); Serial.print(F("\t"));
    Serial.print(angleY, 2); Serial.print(F("\t"));
    Serial.print(moveX);     Serial.print(F("\t"));
    Serial.print(moveY);     Serial.print(F("\t"));
    Serial.print(F("OK:"));  Serial.print(countOK);
    Serial.print(F(" FAIL:")); Serial.println(countFAIL);
    Serial.print(F(" L:"));  Serial.print(data.clickLeft  ? F("1") : F("0"));
    Serial.print(F(" R:"));  Serial.print(data.clickRight ? F("1") : F("0"));
    if (countFAIL > 20) LED_AMARILLO(); else LED_VERDE();
    countOK = 0;
    countFAIL = 0;
  }
}

/* ============================================================
 *  Calibración del MPU6050
 *  Calcula los offsets del giroscopio y la posición neutra de la cabeza,
 *  promediando varias muestras con el sensor en reposo.
 * ============================================================
*/
void calibrarMPU6050() {
  float sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumAX = 0, sumAY = 0, sumAZ = 0;
  int16_t rawAX, rawAY, rawAZ, rawGX, rawGY, rawGZ;

  Serial.print(F("[CALIBRACION] Tomando "));
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(F(" muestras..."));

  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    mpu.getMotion6(&rawAX, &rawAY, &rawAZ, &rawGX, &rawGY, &rawGZ);
    sumAX += rawAX / ACCEL_SCALE;
    sumAY += rawAY / ACCEL_SCALE;
    sumAZ += rawAZ / ACCEL_SCALE;
    sumGX += rawGX / GYRO_SCALE;
    sumGY += rawGY / GYRO_SCALE;
    sumGZ += rawGZ / GYRO_SCALE;
    esperar(2);
  }

  // Promedios.
  float avgAX = sumAX / CALIBRATION_SAMPLES;
  float avgAY = sumAY / CALIBRATION_SAMPLES;
  float avgAZ = sumAZ / CALIBRATION_SAMPLES;
  gyroOffsetX = sumGX / CALIBRATION_SAMPLES;
  gyroOffsetY = sumGY / CALIBRATION_SAMPLES;
  gyroOffsetZ = sumGZ / CALIBRATION_SAMPLES;

  // Ángulos de referencia (posición neutra de la cabeza).
  refAngleX = atan2(avgAY, avgAZ) * 180.0 / PI;
  refAngleY = atan2(-avgAX, avgAZ) * 180.0 / PI;
  angleX = refAngleX;
  angleY = refAngleY;

  Serial.print(F("  Offset Gyro X/Y/Z: "));
  Serial.print(gyroOffsetX, 3); Serial.print(F(" / "));
  Serial.print(gyroOffsetY, 3); Serial.print(F(" / "));
  Serial.println(gyroOffsetZ, 3);
  Serial.print(F("  Angulo neutro X/Y: "));
  Serial.print(refAngleX, 2); Serial.print(F(" / "));
  Serial.println(refAngleY, 2);
}

// ============================================================
//  Zona muerta
//  Dentro de la zona muerta → 0. Fuera → resta la zona muerta para que
//  no haya un salto brusco justo en el límite.
// ============================================================
float applyDeadzone(float value, float deadzone) {
  if (value >  deadzone) return value - deadzone;
  if (value < -deadzone) return value + deadzone;
  return 0.0;
}

// ============================================================
//  Limitar el movimiento al rango [-maxSpeed, +maxSpeed]
// ============================================================
int clampCursorSpeed(float value, int maxSpeed) {
  if (value >  maxSpeed) return maxSpeed;
  if (value < -maxSpeed) return -maxSpeed;
  return (int)value;
}
