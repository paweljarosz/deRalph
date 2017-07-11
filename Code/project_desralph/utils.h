#ifndef __UTILS_H__
#define __UTILS_H__

#include <hFramework.h>
#include "MPU9250.h"

using namespace hModules;

void resetI2C(ISensor_i2c& sens);
bool initMPU(MPU9250& mpu);
void mpuToAccelGyro(MPU9250& mpu, float& accel, float& gyro);

int32_t getPosition();

void openLeg();
void closeLeg();
void releaseLeg();

#endif
