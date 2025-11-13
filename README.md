# ESP DRONE
*Authot:* RUSU MIHNEA.

This is a project based on a esp32-c3 super mini and esp-now protocol.
## Materials used in

**IMU:** 6 axis MPU6050 accelerometer and gyroscope\
**Motor Driver:** A self made driver using AO3400 N channel mosfet, 1 1k 0805 resistor and BAT60B diode.\
**Motor:** 4 high speed brushed motors\
**uC:** Esp32 C3\
**Charging Module:** TP4057 Mini TuanNH data\
**Battery:** High Discharge 25C with 600mAh

## Code Components

**Complementary Filter:** Used with the MPU6050 to merge the accelerometer and gyrocope in order to obtain a good reading of the angle\
**PID Controllers:** Used to stabilize the drone > 1 for every axis\
**Mixing Algorithm:** Takes the PID outputs and mix the values so that the motors recive the correct PWM value


