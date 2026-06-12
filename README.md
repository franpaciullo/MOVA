# MOVA - Mouse Giroscópico Inalámbrico

Proyecto Tecnológico Final 2026

COMPONENTES
------------------------------------------------------

 -2x Arduino Pro Micro (5V, 16KHz)

 -Giroscópio MPU6050 6 ejes

 -2x NRF24L01 HW-237.

 -Motor vibrador pequeño.

 -Resistencia 100ohm para motor.

 -3x TD1117V33 (u otros STEP-DOWN a 3.3v, para RFs y vibrador).

 -2x Capacitor 47uf para módulos RFs.

 -2x Sensor táctil Tpt223 (clicks).


MOVA-TEST-CODE
------------------------------------------------------

Prueba  de comunicación del Arduino Pro Micro enviando los datos del MPU6050 a través de USB. No usa transmisión RF.


CÓDIGO EMISOR
------------------------------------------------------

Arduino Pro Micro (emisor) envia datos de el MPU6050 y los Tpt223 al receptor a través de RF.


CÓDIGO RECEPTOR
------------------------------------------------------

Arduino Pro Micro (receptor) recibe los datos del emisor a través de RF y mueve el cursor en pantalla a traves de USB



