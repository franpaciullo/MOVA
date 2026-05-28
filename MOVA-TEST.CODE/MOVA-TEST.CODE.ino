#include <Wire.h>
#include <Mouse.h>

/*
 * ============================================================
 *  HEAD TRACKER MOUSE — Arduino Pro Micro + MPU6050 GY-521
 * ============================================================
 *
 * DESCRIPCIÓN:
 *   Convierte movimientos de cabeza detectados por el MPU6050
 *   en movimiento real del cursor del mouse en Windows, usando
 *   el Arduino Pro Micro como dispositivo HID USB.
 *
 * CONEXIONES (Pro Micro <-> MPU6050 GY-521):
 * ┌─────────────────┬──────────────┬─────────────────────────┐
 * │  Pro Micro Pin  │  GY-521 Pin  │  Descripción            │
 * ├─────────────────┼──────────────┼─────────────────────────┤
 * │  VCC (5V o 3V3) │  VCC         │  Alimentación (5V OK)   │
 * │  GND            │  GND         │  Tierra común           │
 * │  Pin 2 (SDA)    │  SDA         │  I2C Datos              │
 * │  Pin 3 (SCL)    │  SCL         │  I2C Reloj              │
 * │  (sin conectar) │  AD0         │  Dejar libre (addr 0x68)│
 * │  (sin conectar) │  INT         │  No se usa en este código│
 * └─────────────────┴──────────────┴─────────────────────────┘
 *
 * NOTAS DE CONEXIÓN:
 *  - En el Pro Micro los pines I2C son: SDA = Pin 2, SCL = Pin 3
 *  - El pin AD0 del MPU6050 libre (o a GND) → dirección I2C: 0x68
 *  - Si AD0 está en VCC → dirección I2C: 0x69 (cambiar MPU_ADDR abajo)
 *  - El GY-521 ya tiene resistencias pull-up en SDA/SCL integradas
 *
 * LIBRERÍAS REQUERIDAS:
 *  - Wire.h   (incluida en Arduino IDE)
 *  - Mouse.h  (incluida para ATmega32U4 — Pro Micro, Leonardo)
 *
 * COMPILACIÓN:
 *  - Board: "SparkFun Pro Micro" o "Arduino Leonardo"
 *  - Processor: ATmega32U4 (5V, 16MHz)
 *
 * AUTOR: Generado para uso personal
 * ============================================================
 */



// ============================================================
//  DIRECCIÓN I2C DEL MPU6050
// ============================================================
#define MPU_ADDR 0x68  // AD0=GND → 0x68 | AD0=VCC → 0x69

// ============================================================
//  REGISTROS DEL MPU6050
// ============================================================
#define MPU_PWR_MGMT_1   0x6B  // Registro de gestión de energía
#define MPU_ACCEL_CONFIG  0x1C  // Configuración del acelerómetro
#define MPU_GYRO_CONFIG   0x1B  // Configuración del giroscopio
#define MPU_DLPF_CONFIG   0x1A  // Filtro paso bajo digital
#define MPU_ACCEL_XOUT_H  0x3B  // Primer registro de datos acelerómetro
#define MPU_GYRO_XOUT_H   0x43  // Primer registro de datos giroscopio
 
// ============================================================
//  CONFIGURACIÓN — Ajusta estos valores según tu preferencia
// ============================================================

// --- Sensibilidad del cursor ---
// Valores más altos = movimiento más rápido del cursor
// Rango recomendado: 5.0 a 20.0
const float SENSITIVITY_X = 1.0;  // Sensibilidad horizontal
const float SENSITIVITY_Y = 1.0;  // Sensibilidad vertical

// --- Zona muerta (deadzone) ---
// Ángulo mínimo de inclinación para mover el cursor (en grados)
// Elimina el jitter cuando la cabeza está quieta
// Rango recomendado: 0.5 a 2.5
const float DEADZONE = 3;

// --- Filtro complementario ---
// Alpha cercano a 1.0 confía más en el giroscopio (más suave, más drift)
// Alpha cercano a 0.0 confía más en el acelerómetro (menos drift, más ruido)
// Rango recomendado: 0.90 a 0.98
const float ALPHA = 0.96;

// --- Suavizado de movimiento (media móvil exponencial) ---
// Smoothing cercano a 1.0 = muy suave pero lento de responder
// Smoothing cercano a 0.0 = respuesta rápida pero más brusco
// Rango recomendado: 0.50 a 0.85
const float SMOOTHING = 0.70;

// --- Velocidad máxima del cursor (píxeles por ciclo) ---
// Limita el movimiento máximo para evitar saltos bruscos
const int MAX_CURSOR_SPEED = 20;

// --- Intervalo de loop en ms ---
// Tiempo entre lecturas. 10ms = ~100Hz
const int LOOP_INTERVAL_MS = 10;

// --- Número de muestras para calibración inicial ---
const int CALIBRATION_SAMPLES = 500;

// ============================================================
//  ESCALAS DEL MPU6050 (según configuración de registros)
// ============================================================
// Acelerómetro: ±2g → LSB/g = 16384.0
const float ACCEL_SCALE = 16384.0;
// Giroscopio: ±250°/s → LSB/(°/s) = 131.0
const float GYRO_SCALE  = 131.0;

// ============================================================
//  VARIABLES GLOBALES
// ============================================================

// Offsets de calibración del giroscopio (se calculan al inicio)
float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float gyroOffsetZ = 0.0;

// Ángulos calculados por el filtro complementario (grados)
float angleX = 0.0;  // Inclinación lateral  (roll)  → mueve cursor Y
float angleY = 0.0;  // Inclinación frontal  (pitch) → mueve cursor X

// Ángulos de referencia (posición neutra de la cabeza)
float refAngleX = 0.0;
float refAngleY = 0.0;

// Movimiento suavizado del cursor
float smoothMouseX = 0.0;
float smoothMouseY = 0.0;

// Marcador de tiempo para control de loop
unsigned long lastTime = 0;

// Marcador de tiempo para Serial Monitor
unsigned long lastSerialPrint = 0;
const int SERIAL_INTERVAL_MS = 100;  // Imprimir cada 100ms

// ============================================================
//  PROTOTIPOS DE FUNCIONES
// ============================================================
void    initMPU6050();
void    calibrateMPU6050();
void    readMPU6050(float &ax, float &ay, float &az,
                    float &gx, float &gy, float &gz);
int16_t readRegister16(uint8_t reg);
float   applyDeadzone(float value, float deadzone);
int     clampCursorSpeed(float value, int maxSpeed);
void    printSerialData(float ax, float ay, float gx, float gy,
                        float aX, float aY, int mX, int mY);

// ============================================================
//  SETUP
// ============================================================
void setup() {

  // Iniciar comunicación serial para debug
  Serial.begin(115200);
  // Esperar a que Serial esté listo (necesario en ATmega32U4)
  // Timeout de 3 segundos para no bloquear si no hay monitor serial
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 3000)) {
    delay(10);
  }

  Serial.println(F("============================================"));
  Serial.println(F("  MOUSE GIROSCOPICO INALAMBRICO — Iniciando..."));
  Serial.println(F("============================================"));

  // Iniciar bus I2C
  Wire.begin();
  Wire.setClock(400000);  // Modo Fast I2C: 400kHz para mayor velocidad

  // Inicializar y configurar el MPU6050
  initMPU6050();

  // Calibrar el giroscopio (cabeza quieta y plana)
  Serial.println(F("\n[CALIBRACION] Mantener cabeza QUIETA y mirando al frente..."));
  Serial.println(F("[CALIBRACION] Iniciando en 2 segundos..."));
  delay(2000);
  calibrateMPU6050();
  Serial.println(F("[CALIBRACION] Completada!\n"));

  // Iniciar el Mouse HID
  Mouse.begin();
  Serial.println(F("[MOUSE] Dispositivo HID Mouse activado."));
  Serial.println(F("[MOUSE] Mover la cabeza para controlar el cursor.\n"));

  // Encabezado del Serial Monitor
  Serial.println(F("accelX\taccelY\tgyroX\tgyroY\tangleX\tangleY\tmouseX\tmouseY"));

  // Guardar tiempo inicial
  lastTime = millis();
  lastSerialPrint = millis();
}

// ============================================================
//  LOOP PRINCIPAL
// ============================================================
void loop() {

  unsigned long currentTime = millis();

  // Control de frecuencia: ejecutar solo cada LOOP_INTERVAL_MS
  if ((currentTime - lastTime) < LOOP_INTERVAL_MS) {
    return;
  }

  // Delta de tiempo en segundos (para integración del giroscopio)
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;

  // Limitar dt para evitar saltos grandes si el loop se retrasa
  if (dt > 0.05) dt = 0.05;

  // --- 1. LEER SENSORES ---
  float ax, ay, az;  // Acelerómetro (en g)
  float gx, gy, gz;  // Giroscopio (en °/s)
  readMPU6050(ax, ay, az, gx, gy, gz);

  // --- 2. FILTRO COMPLEMENTARIO ---
  // Ángulo del acelerómetro (referencia estática, sin drift)
  // atan2 devuelve radianes → convertir a grados
  float accelAngleX = atan2(ay, az) * 180.0 / PI;  // Roll
  float accelAngleY = atan2(-ax, az) * 180.0 / PI; // Pitch

  // Integrar el giroscopio (rápido, sin ruido, con drift lento)
  // y fusionar con el acelerómetro mediante filtro complementario
  angleX = ALPHA * (angleX + gx * dt) + (1.0 - ALPHA) * accelAngleX;
  angleY = ALPHA * (angleY + gy * dt) + (1.0 - ALPHA) * accelAngleY;

  // --- 3. CALCULAR DESPLAZAMIENTO RELATIVO A LA POSICIÓN NEUTRA ---
  float deltaX = angleX - refAngleX;  // Desviación en roll  → cursor Y
  float deltaY = angleY - refAngleY;  // Desviación en pitch → cursor X

  // --- 4. APLICAR ZONA MUERTA ---
  float deadDeltaX = applyDeadzone(deltaX, DEADZONE);
  float deadDeltaY = applyDeadzone(deltaY, DEADZONE);

  // --- 5. CALCULAR MOVIMIENTO DEL CURSOR ---
  // deltaY (pitch = cabeza arriba/abajo) controla Y del cursor
  // deltaX (roll  = cabeza izquierda/derecha) controla X del cursor
  // Nota: el signo determina la dirección natural del movimiento
  float rawMouseX =  deadDeltaY * SENSITIVITY_X;  // Pitch → X
  float rawMouseY = -deadDeltaX * SENSITIVITY_Y;  // Roll  → Y (invertido para naturalidad)

  // --- 6. SUAVIZADO (Media Móvil Exponencial) ---
  smoothMouseX = SMOOTHING * smoothMouseX + (1.0 - SMOOTHING) * rawMouseX;
  smoothMouseY = SMOOTHING * smoothMouseY + (1.0 - SMOOTHING) * rawMouseY;

  // --- 7. LIMITAR VELOCIDAD MÁXIMA ---
  int moveX = clampCursorSpeed(smoothMouseX, MAX_CURSOR_SPEED);
  int moveY = clampCursorSpeed(smoothMouseY, MAX_CURSOR_SPEED);

  // --- 8. MOVER EL CURSOR (solo si hay movimiento real) ---
  if (moveX != 0 || moveY != 0) {
    Mouse.move(moveX, moveY, 0);
  }

  // --- 9. IMPRIMIR DATOS EN SERIAL MONITOR ---
  if ((currentTime - lastSerialPrint) >= SERIAL_INTERVAL_MS) {
    printSerialData(ax, ay, gx, gy, angleX, angleY, moveX, moveY);
    lastSerialPrint = currentTime;
  }
}

// ============================================================
//  FUNCIÓN: Inicializar MPU6050
// ============================================================
void initMPU6050() {

  Serial.println(F("[MPU6050] Inicializando..."));

  // Despertar el MPU6050 (por defecto está en modo sleep)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_PWR_MGMT_1);
  Wire.write(0x00);  // Bit SLEEP = 0, usar oscilador interno
  Wire.endTransmission(true);
  delay(100);

  // Verificar que el MPU6050 responde (registro WHO_AM_I = 0x75)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);  // WHO_AM_I register
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  uint8_t whoAmI = Wire.read();

  if (whoAmI == 0x68) {
    Serial.println(F("[MPU6050] Detectado correctamente (WHO_AM_I = 0x68)"));
  } else {
    Serial.print(F("[MPU6050] ADVERTENCIA: WHO_AM_I = 0x"));
    Serial.println(whoAmI, HEX);
    Serial.println(F("[MPU6050] Verificar conexiones I2C y dirección."));
  }

  // Configurar filtro paso-bajo digital (DLPF)
  // DLPF_CFG = 3 → Accel: 44Hz BW, Gyro: 42Hz BW, delay ~4.8ms
  // Reduce ruido de alta frecuencia para movimiento más suave
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_DLPF_CONFIG);
  Wire.write(0x03);  // DLPF nivel 3
  Wire.endTransmission(true);

  // Configurar rango del giroscopio: ±250°/s (más precisión para movimientos suaves)
  // FS_SEL = 0 → ±250°/s → escala 131 LSB/(°/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_GYRO_CONFIG);
  Wire.write(0x00);  // FS_SEL = 0 → ±250°/s
  Wire.endTransmission(true);

  // Configurar rango del acelerómetro: ±2g (máxima resolución)
  // AFS_SEL = 0 → ±2g → escala 16384 LSB/g
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_CONFIG);
  Wire.write(0x00);  // AFS_SEL = 0 → ±2g
  Wire.endTransmission(true);

  delay(100);
  Serial.println(F("[MPU6050] Configurado: Gyro ±250°/s | Accel ±2g | DLPF nivel 3"));
}

// ============================================================
//  FUNCIÓN: Calibración automática del MPU6050
//  Calcula los offsets promedio del giroscopio y
//  establece la posición neutra de la cabeza (angleX, angleY)
// ============================================================
void calibrateMPU6050() {

  float sumGX = 0, sumGY = 0, sumGZ = 0;
  float sumAX = 0, sumAY = 0, sumAZ = 0;

  Serial.print(F("[CALIBRACION] Tomando "));
  Serial.print(CALIBRATION_SAMPLES);
  Serial.println(F(" muestras..."));

  // Tomar múltiples lecturas para promediar
  for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
    int16_t rawAX = readRegister16(MPU_ACCEL_XOUT_H);
    int16_t rawAY = readRegister16(MPU_ACCEL_XOUT_H + 2);
    int16_t rawAZ = readRegister16(MPU_ACCEL_XOUT_H + 4);
    int16_t rawGX = readRegister16(MPU_GYRO_XOUT_H);
    int16_t rawGY = readRegister16(MPU_GYRO_XOUT_H + 2);
    int16_t rawGZ = readRegister16(MPU_GYRO_XOUT_H + 4);

    sumAX += rawAX / ACCEL_SCALE;
    sumAY += rawAY / ACCEL_SCALE;
    sumAZ += rawAZ / ACCEL_SCALE;
    sumGX += rawGX / GYRO_SCALE;
    sumGY += rawGY / GYRO_SCALE;
    sumGZ += rawGZ / GYRO_SCALE;

    // Mostrar progreso cada 100 muestras
    if ((i + 1) % 100 == 0) {
      Serial.print(F("  ..."));
      Serial.print(i + 1);
      Serial.println(F(" muestras tomadas"));
    }
    delay(2);  // ~500 muestras en ~1 segundo
  }

  // Calcular promedios
  float avgAX = sumAX / CALIBRATION_SAMPLES;
  float avgAY = sumAY / CALIBRATION_SAMPLES;
  float avgAZ = sumAZ / CALIBRATION_SAMPLES;
  gyroOffsetX = sumGX / CALIBRATION_SAMPLES;
  gyroOffsetY = sumGY / CALIBRATION_SAMPLES;
  gyroOffsetZ = sumGZ / CALIBRATION_SAMPLES;

  // Establecer ángulos de referencia (posición neutra de la cabeza)
  refAngleX = atan2(avgAY, avgAZ) * 180.0 / PI;
  refAngleY = atan2(-avgAX, avgAZ) * 180.0 / PI;

  // Inicializar ángulos del filtro en la posición neutra
  angleX = refAngleX;
  angleY = refAngleY;

  // Mostrar resultados de calibración
  Serial.println(F("\n[CALIBRACION] Resultados:"));
  Serial.print(F("  Offset Gyro X: ")); Serial.println(gyroOffsetX, 4);
  Serial.print(F("  Offset Gyro Y: ")); Serial.println(gyroOffsetY, 4);
  Serial.print(F("  Offset Gyro Z: ")); Serial.println(gyroOffsetZ, 4);
  Serial.print(F("  Angulo neutro X (roll):  ")); Serial.println(refAngleX, 2);
  Serial.print(F("  Angulo neutro Y (pitch): ")); Serial.println(refAngleY, 2);
}

// ============================================================
//  FUNCIÓN: Leer todos los datos del MPU6050
//  Retorna acelerómetro en [g] y giroscopio en [°/s],
//  con offsets de calibración ya aplicados.
// ============================================================
void readMPU6050(float &ax, float &ay, float &az,
                 float &gx, float &gy, float &gz) {

  // Solicitar lectura de 14 bytes desde el primer registro de acelerómetro
  // Los registros son consecutivos: ACCEL XYZ (6 bytes) + TEMP (2) + GYRO XYZ (6)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  // Leer acelerómetro (bytes 0-5)
  int16_t rawAX = (Wire.read() << 8) | Wire.read();
  int16_t rawAY = (Wire.read() << 8) | Wire.read();
  int16_t rawAZ = (Wire.read() << 8) | Wire.read();

  // Leer temperatura (bytes 6-7) — no se usa, pero hay que leerlos
  Wire.read(); Wire.read();

  // Leer giroscopio (bytes 8-13)
  int16_t rawGX = (Wire.read() << 8) | Wire.read();
  int16_t rawGY = (Wire.read() << 8) | Wire.read();
  int16_t rawGZ = (Wire.read() << 8) | Wire.read();

  // Convertir a unidades físicas y aplicar offsets de calibración
  ax = rawAX / ACCEL_SCALE;
  ay = rawAY / ACCEL_SCALE;
  az = rawAZ / ACCEL_SCALE;

  gx = (rawGX / GYRO_SCALE) - gyroOffsetX;
  gy = (rawGY / GYRO_SCALE) - gyroOffsetY;
  gz = (rawGZ / GYRO_SCALE) - gyroOffsetZ;
}

// ============================================================
//  FUNCIÓN: Leer un registro de 16 bits del MPU6050
//  Usada durante la calibración para leer registros individuales
// ============================================================
int16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);
  return (Wire.read() << 8) | Wire.read();
}

// ============================================================
//  FUNCIÓN: Aplicar zona muerta (deadzone)
//  Si el valor absoluto está dentro de la zona muerta, retorna 0.
//  Si supera la zona muerta, resta la zona muerta del valor
//  (para evitar el salto brusco en el límite).
// ============================================================
float applyDeadzone(float value, float deadzone) {
  if (value > deadzone) {
    return value - deadzone;   // Supera zona muerta positiva
  } else if (value < -deadzone) {
    return value + deadzone;   // Supera zona muerta negativa
  }
  return 0.0;  // Dentro de la zona muerta → sin movimiento
}

// ============================================================
//  FUNCIÓN: Limitar velocidad máxima del cursor
//  Convierte float a int limitado al rango [-maxSpeed, +maxSpeed]
// ============================================================
int clampCursorSpeed(float value, int maxSpeed) {
  if (value > maxSpeed)  return maxSpeed;
  if (value < -maxSpeed) return -maxSpeed;
  return (int)value;
}

// ============================================================
//  FUNCIÓN: Imprimir datos en Serial Monitor
//  Formato tabular para facilitar lectura y graficación
// ============================================================
void printSerialData(float ax, float ay, float gx, float gy,
                     float aX, float aY, int mX, int mY) {
  Serial.print(ax,  3);  Serial.print(F("\t"));
  Serial.print(ay,  3);  Serial.print(F("\t"));
  Serial.print(gx,  3);  Serial.print(F("\t"));
  Serial.print(gy,  3);  Serial.print(F("\t"));
  Serial.print(aX,  2);  Serial.print(F("\t"));
  Serial.print(aY,  2);  Serial.print(F("\t"));
  Serial.print(mX);      Serial.print(F("\t"));
  Serial.println(mY);
}

/*
 * ============================================================
 *  GUÍA RÁPIDA DE AJUSTE
 * ============================================================
 *
 * El cursor se mueve demasiado rápido:
 *   → Reducir SENSITIVITY_X y SENSITIVITY_Y (ej: 6.0)
 *
 * El cursor tiembla o vibra cuando la cabeza está quieta:
 *   → Aumentar DEADZONE (ej: 1.8)
 *
 * El cursor tarda mucho en responder:
 *   → Reducir SMOOTHING (ej: 0.55)
 *   → Aumentar SENSITIVITY (ej: 14.0)
 *
 * El cursor se mueve lentamente o con retraso:
 *   → Reducir SMOOTHING (ej: 0.50)
 *
 * El cursor deriva lentamente con la cabeza quieta (drift):
 *   → Reducir ALPHA (ej: 0.92) para confiar más en acelerómetro
 *   → Asegurarse de que la calibración se hizo con la cabeza QUIETA
 *
 * Movimiento se siente brusco:
 *   → Aumentar SMOOTHING (ej: 0.80)
 *
 * ============================================================
 *  SOLUCIÓN DE PROBLEMAS
 * ============================================================
 *
 * "WHO_AM_I" incorrecto en Serial Monitor:
 *   → Verificar conexiones SDA/SCL
 *   → Confirmar que VCC está conectado
 *   → Si AD0 está en VCC, cambiar MPU_ADDR a 0x69
 *
 * Arduino no se detecta como mouse en Windows:
 *   → Asegurarse de seleccionar "SparkFun Pro Micro" o
 *     "Arduino Leonardo" como placa en Arduino IDE
 *   → Si antes era detectado como Serial, puede necesitar
 *     re-flash del bootloader
 *
 * El cursor solo se mueve en una dirección:
 *   → Revisar la orientación física del MPU6050
 *   → Cambiar el signo en rawMouseX o rawMouseY
 *
 * ============================================================
 */
