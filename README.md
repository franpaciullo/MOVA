# MOVA - Mouse Giroscópico Inalámbrico

Proyecto Tecnológico Final 2026

COMPONENTES
------------------------------------------------------

 -2x Arduino Pro Micro (5V, 16KHz)

 -Giroscópio MPU6050 6 ejes

 -2x NRF24L01 MW-237

 -Motor vibrador pequeño

 -Resistencia 47ohm para motor.

 -3x TD1117V33 (STEP-DOWN a 3.3v para RFs y vibrador)

 -2x Capacitor 47uf para módulos RFs.

 -Sensor de proximidad Gy 9960 ó Sensor táctil Tpt223 (para clicks, a definir)


MOVA-TEST-CODE
------------------------------------------------------

Prueba comunicación del Arduino Pro Micro enviando los datos del MPU6050 a través de USB. No usa transmisión RF.


CÓDIGO EMISOR
------------------------------------------------------

Actualmente no utiliza vibrador ni sensores para clicks.
Arduino Pro Micro (emisor) envia datos del MPU6050 al receptor. 


CÓDIGO RECEPTOR
------------------------------------------------------

Arduino Pro Micro (receptor) recibe los datos del emisor a través de RF y mueve el cursor en pantalla a traves de USB



