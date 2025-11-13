void GetValues(MPU6050 *mpu, float *RotationVector);
void FilterValues(float *RotationVector);
void CombineSensors(MPU6050 *mpu, float *RotationVector);