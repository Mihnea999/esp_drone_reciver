
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define FLM 5
#define FRM 6
#define RRM 3
#define RLM 1

typedef struct struct_message
{
  int8_t Throttle;
  int8_t Rotate;
  int8_t Roll;
  int8_t Pitch;

  bool lb;
  bool rb;

  int8_t _kp;
  int8_t _kd;
  int8_t _ki;

} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));

  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("Throttle: ");
  // Serial.println(myData.Throttle);
  // Serial.print("Rotate: ");
  // Serial.println(myData.Rotate / 100.00);
  // Serial.print("Roll: ");
  // Serial.println(myData.Roll / 100.00);
  // Serial.print("Pitch: ");
  // Serial.println(myData.Pitch / 100.00);
  // Serial.print("LB: ");
  // Serial.println(myData.lb);
  // Serial.print("RB: ");
  // Serial.println(myData.rb);

  Serial.println();
}

void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
    while (1)
    {
      yield();
      Serial.println("ERR ESP NOW\n");
    }
  }
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0)
  {
    Serial.println("ERR");
  }
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true, true); // gyro and accelero
  Serial.println("Done!\n");

  Serial.println("");
  pinMode(RLM, OUTPUT);
  pinMode(RRM, OUTPUT);
  pinMode(FRM, OUTPUT);
  pinMode(FLM, OUTPUT);
  analogWriteFrequency(23000);
  delay(100);
}
// prototyping
float RotationVector[3];
float RollCorection();
float PitchCorection();
float RotationCorection();
void OutputAlg(float pc, float rc, float rotc);

#include "sensorprocessing.h"

void loop()
{
  // Serial.println(WiFi.softAPIP());
  GetValues(&mpu, RotationVector);
  // Serial.print("X: ");
  // Serial.println(RotationVector[0] * 90);
  // Serial.print("Correction on X: ");
  // Serial.print(RollCorection());
  // Serial.print("    ");
  // Serial.print(", Y: ");
  // Serial.print(RotationVector[1]);
  // Serial.print("Correction on Y: ");
  // Serial.print(PitchCorection());
  // Serial.println();
  OutputAlg(PitchCorection(), RollCorection(), RotationCorection());
  // Serial.print(", Z: ");
  // Serial.print(RotationVector[2]);
  // Serial.println(" %");

  // Serial.print("Rotation X: ");
  // Serial.print(g.gyro.x);
  // Serial.print(", Y: ");
  // Serial.print(g.gyro.y);
  // Serial.print(", Z: ");
  // Serial.print(mpu.getGyroZ());
  // Serial.println(" rad/s");

  // Serial.print("Temperature: ");
  // Serial.print(temp.temperature);
  // Serial.println(" degC");

  // analogWrite(FLM, 50);
  // analogWrite(FRM, 50);
  // analogWrite(RLM, 50);
  // analogWrite(RRM, 50);
}
unsigned long old_time1 = 1;
unsigned long old_time2 = 1;
unsigned long old_time3 = 1;

float integrativeTermRC = 0;
float OldErrorRC = 0;
float RollCorection()
{
  float kp = myData._kp / 10.0;
  float ki = myData._ki / 100.0;
  float kd = myData._kd / 100.0;

  float SetValue = 0;
  float Value = RotationVector[0];

  float Error = SetValue - Value;

  unsigned long time = millis() - old_time1;

  float pp = kp * Error;
  float pd = kd * ((Error - OldErrorRC) / time);
  integrativeTermRC += ki * (Error * time);

  if (integrativeTermRC > 15)
    integrativeTermRC = 15;
  if (integrativeTermRC < -15)
    integrativeTermRC = -15;
  old_time1 = millis();
  OldErrorRC = Error;

  return pp + pd + integrativeTermRC;
}
float integrativeTermPC = 0;
float OldErrorPC = 0;
float PitchCorection()
{
  float kp = 0;
  float ki = 0;
  float kd = 0;

  float SetValue = 0;
  float Value = RotationVector[1];

  float Error = SetValue - Value;

  unsigned long time = millis() - old_time2;

  float pp = kp * Error;
  float pd = kd * ((Error - OldErrorPC) / time);
  integrativeTermPC += ki * (Error * time / 1000);

  if (integrativeTermPC > 20)
    integrativeTermPC = 20;
  old_time2 = millis();
  OldErrorPC = Error;

  return pp + pd + integrativeTermPC;
}

float OldErrorROC = 0;
float RotationCorection()
{
  float kp = 0;
  float kd = 0;

  float SetValue = 0;
  float Value = (int16_t)RotationVector[2];

  float Error = SetValue - Value;

  unsigned long time = millis() - old_time3;

  float pp = kp * Error;
  float pd = kd * ((Error - OldErrorROC) / time);

  OldErrorROC = Error;
  old_time3 = millis();

  return pp + pd;
}
void OutputAlg(float pc, float rc, float rotc) // pc=Pitch, rc=Roll, rotc=Yaw
{
  // m1 - FL | m2 - FR
  // m3 - RL | m4 - RR

  // 1. Throttle-ul ca bază
  int16_t m1, m2, m3, m4;
  m1 = m2 = m3 = m4 = 2 * myData.Throttle;
  // Serial.printf("FL-%d  FR-%d  RL-%d  RR-%d\n", m1, m2, m3, m4);
  //  2. Aplică Amestecul (Mixing)

  // Corecția Roll (rc): M1, M3: Stanga; M2, M4: Dreapta
  m1 += rc; // Stanga (M1) creste daca rc e negativ (inclinare la dreapta)
  m3 += rc; // Stanga (M3) creste daca rc e negativ
  m2 -= rc; // Dreapta (M2) scade daca rc e negativ
  m4 -= rc; // Dreapta (M4) scade daca rc e negativ

  // // Corecția Pitch (pc): M1, M2: Fata; M3, M4: Spate
  m1 += pc; // Fata (M1) scade daca pc e negativ (inclinare in fata)
  m2 += pc; // Fata (M2) scade daca pc e negativ
  m3 -= pc; // Spate (M3) creste daca pc e negativ
  m4 -= pc; // Spate (M4) creste daca pc e negativ

  // // Corecția Yaw (rotc): M1, M4 vs M2, M3
  m1 += rotc; // Diagonala 1
  m4 += rotc; // Diagonala 1
  m2 -= rotc; // Diagonala 2
  m3 -= rotc; // Diagonala 2
  // // Limitează la 0 - 254
  m1 = constrain(m1, 0, 254);
  m2 = constrain(m2, 0, 254);
  m3 = constrain(m3, 0, 254);
  m4 = constrain(m4, 0, 254);

  // Serial.printf("FL-%d  FR-%d  RL-%d  RR-%d\n", m1, m2, m3, m4);

  analogWrite(FLM, (myData.Throttle > 15) ? m1 : 0);
  analogWrite(FRM, (myData.Throttle > 15) ? m2 : 0);
  analogWrite(RLM, (myData.Throttle > 15) ? m3 : 0);
  analogWrite(RRM, (myData.Throttle > 15) ? m4 : 0);
}