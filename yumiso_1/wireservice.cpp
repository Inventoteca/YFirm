#include "wireservice.h"

Adafruit_MPU6050 mpu;
Servo servo1;

bool moved = false;
int timeServoOn = 2000;
int counterServoOn;


// ------------------------------------ initMotion
void InitMotion()
{
  byte error, address;
  int nDevices = 0;

  //Wire.setSDA(SDA_MAIN);
  //Wire.setSCL(SCL_MAIN);
  Wire.begin();
  servo1.attach(servo1Pin);
  servo1.write(SERVO_TEST);
  delay(1000);
  Serial.println("Scanning for I2C devices ...");
  for (address = 0x01; address < 0x7f; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("I2C device found at address 0x%02X\n", address);
      nDevices++;
    } else if (error != 2) {
      Serial.printf("Error %d at address 0x%02X\n", error, address);
    }
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  }

  if (mpu.begin())
  {
    Serial.println("{\"accelInit\":true}");
    status_doc["accelInit"] = true;

    //setupt motion detection
    mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
    mpu.setMotionDetectionThreshold(5);
    mpu.setMotionDetectionDuration(20);
    mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
    mpu.setInterruptPinPolarity(true);
    mpu.setMotionInterrupt(true);

    Serial.println("");
  }
  else
  {
    Serial.println("{\"accelInit\":false}");
    status_doc["accelInit"] = false;
  }


  servo1.write(SERVO_OPEN);
  //delay(1000);
  //servo1.detach();
}


// ---------------------------------------- GetMotion
void GetMotion()
{
  if (mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    servo1.attach(servo1Pin);
    servo1.write(SERVO_CLOSE);

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");

    //delay(1000);
    //servo1.detach();

    moved = true;
    counterServoOn = 0;
  }
}



// ------------------------------------------GetAngle
void GetAngle()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;

  float roll = atan2(ay, az) * 180 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;





  if ((abs(roll) > 10 ) || (abs(pitch) > 10 ))
  {

    servo1.attach(servo1Pin);
    servo1.write(SERVO_CLOSE);

    Serial.print("Roll: ");
    Serial.print(roll);
    Serial.print(" Pitch: ");
    Serial.println(pitch);

    moved = true;
    counterServoOn = 0;
  }

  if ((moved == true) || (servo1.read() == SERVO_TEST) || (servo1.read() == SERVO_OPEN))
  {
    Serial.println("{\"moved\":true}");
    counterServoOn++;
    if (counterServoOn >= 2)
    {
      servo1.write(SERVO_OPEN);
    }
    if ((counterServoOn >= 3) || (servo1.read() == SERVO_OPEN))
    {
      moved = false;
      counterServoOn = 0;
      servo1.detach();
    }
  }
}
