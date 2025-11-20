# ESP DRONE

**Author**: RUSU MIHNEA

This is a project based on an **ESP32-C3 Super Mini** and the **ESP-NOW** protocol for wireless communication.

## Hardware Components

| Component | Details |
| :--- | :--- |
| **IMU** | 6-axis **MPU6050** accelerometer and gyroscope. |
| **Motor Driver** | A custom-made driver using **AO3400 N-channel MOSFET**, 1kΩ 0805 resistor, and **BAT60B** diode. |
| **Motor** | 4 x High-speed brushed motors. |
| **uC (Microcontroller)** | **ESP32-C3**. |
| **Charging Module** | **TP4057 Mini TuanNH data**. |
| **Battery** | High Discharge **25C** with **600mAh**. |

---

## Code Components

### **Complementary Filter**
Used with the MPU6050 to merge the accelerometer and gyroscope data in order to obtain a reliable reading of the angle.

### **PID Controllers**
Used to stabilize the drone. A dedicated **PID controller** is implemented for every axis (Roll, Pitch, Yaw).

### **Mixing Algorithm**
Takes the PID outputs and mixes the values so that the motors receive the correct **PWM** value for thrust control.

---

## BOM

|ID |Name                   |Designator |Footprint                        |Quantity|Manufacturer Part  |
|---|-----------------------|-----------|---------------------------------|--------|-------------------|
|1  |4.7kΩ                  |R5,R6      |R0805                            |2       |0805W8F1001T5E     |
|2  |ESP32-C3-SUPER-MINI    |U1         |ESP32-C3-SUPER-MINI              |1       |                   |
|11 |470uF                  |C1         |CAP-TH_BD6.3-P2.50-D0.5-FD       |1       |GR1C471M0611       |
|12 |100nF                  |C2,C3,C4,C5|C0805                            |4       |C2012X5R1C105M085AA|
|13 |1kΩ                    |R1,R2,R3,R4|R0805                            |4       |0805W8F1001T5E     |
|14 |imu (mpu-6050)         |U2         |MPU-6050                         |1       |                   |
|15 |BAT60B                 |D1,D2,D3,D4|SOD-323_L1.7-W1.3-LS2.7-RD       |4       |1N4148WS           |
|16 |AO3400                 |Q1,Q2,Q3,Q4|SOT-23-3_L3.0-W1.7-P0.95-LS2.9-BR|4       |AO3400             |
|17 |Switch                 |SWITCH     |SW-TH_SS12D00G3                  |1       |SS12D00G3          |
|18 |TP4057 Mini TuanNH data|U4         |TP4057 MINI TUANNH               |1       |TP4057 (TP4056)    |

---

### **[Remote Repository](https://github.com/Mihnea999/ESP_DRONE_TRANSMITER/tree/main)**

---

## PCB DESIGN
<img src="./images/Schematic_dronemainb_2025-11-19.svg">




