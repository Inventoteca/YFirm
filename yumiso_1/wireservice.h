#ifndef  WIRESERVICE_H
#define WIRESERVICE_H

#include "system.h"


#define SERVO_OPEN    0
#define SERVO_CLOSE   90
#define SERVO_TEST   180

#define SDA_MAIN    16
#define SCL_MAIN    17

void InitMotion();
void GetMotion();
void GetAngle();

extern Adafruit_MPU6050 mpu;
extern Servo servo1;

extern bool moved;
extern int timeServoOn;
extern int counterServoOn;

#endif  // 
