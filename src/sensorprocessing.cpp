#include <MPU6050_light.h>
#include "sensorprocessing.h"
const float ALPHA_SMOOTHING = 0.3;

float filteredX = 0.0;
float filteredY = 0.0;
float filteredZ = 0.0;
void FilterValues(float *RotationVector)
{
    filteredX = ALPHA_SMOOTHING * RotationVector[0] + (1.0 - ALPHA_SMOOTHING) * filteredX;
    RotationVector[0] = filteredX;

    filteredY = ALPHA_SMOOTHING * RotationVector[1] + (1.0 - ALPHA_SMOOTHING) * filteredY;
    RotationVector[1] = filteredY;

    filteredZ = ALPHA_SMOOTHING * RotationVector[2] + (1.0 - ALPHA_SMOOTHING) * filteredZ;
    RotationVector[2] = filteredZ;
}

float XValue = 0;
float XOutput = 0;
float YValue = 0;
float YOutput = 0;
uint alpha_trust = 0.78;
unsigned int ot = 1;
void CombineSensors(MPU6050 *mpu, float *RotationVector)
{
    float deltaTime = (millis() - ot) / 1000.0;

    // XValue += mpu->getGyroX() * deltaTime;
    XValue = XOutput + mpu->getGyroX() * deltaTime;
    XOutput = XValue * alpha_trust + RotationVector[0] * (1 - alpha_trust);
    RotationVector[0] = XOutput;

    YValue = YOutput + mpu->getGyroY() * deltaTime;
    YOutput = YValue * alpha_trust + RotationVector[1] * (1 - alpha_trust);
    RotationVector[1] = YOutput;
    ot = millis();
}
void GetValues(MPU6050 *mpu, float *RotationVector)
{
    mpu->update();

    float accY = mpu->getAccY();
    float accZ = -(mpu->getAccZ() - 2);
    float accX = mpu->getAccX();

    // RotationVector[0] = atan2(accY, accZ) * 180.0 / PI;
    // RotationVector[1] = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
    // RotationVector[2] = mpu->getGyroZ();

    //  FilterValues(RotationVector);
    RotationVector[0] = atan2(accX, sqrt(pow(accY, 2.0) + pow(accZ, 2.0))) * (180.0 / 3.14159);
    RotationVector[1] = atan2(accY, sqrt(pow(accX, 2.0) + pow(accZ, 2.0))) * (180.0 / 3.14159);

    CombineSensors(mpu, RotationVector);
    FilterValues(RotationVector);
    Serial.printf("X:  %f  ", RotationVector[0]);
    Serial.printf("Y:  %f  \n", RotationVector[1]);

    // Serial.printf("Y:  %f  ", RotationVector[1]);
    // Serial.printf("Z:  %f  \n", accZ);
}