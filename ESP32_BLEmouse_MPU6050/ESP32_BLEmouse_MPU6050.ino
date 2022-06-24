#include "I2Cdev.h"
#include <MPU6050.h>
#include <BleMouse.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

BleMouse bleMouse;
#define RESTRICT_PITCH 
#define _Kalman_h_

class Kalman {
public:
    Kalman();

    // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
    float getAngle(float newAngle, float newRate, float dt);

    void setAngle(float angle); // Used to set angle, this should be set as the starting angle
    float getRate(); // Return the unbiased rate

    /* These are used to tune the Kalman filter */
    void setQangle(float Q_angle);
    /**
     * setQbias(float Q_bias)
     * Default value (0.003f) is in Kalman.cpp. 
     * Raise this to follow input more closely,
     * lower this to smooth result of kalman filter.
     */
    void setQbias(float Q_bias);
    void setRmeasure(float R_measure);

    float getQangle();
    float getQbias();
    float getRmeasure();

private:
    /* Kalman filter variables */
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

    float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
    float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};


Kalman kalmanY; // Create the Kalman instances
Kalman kalmanX;

/* IMU Data */
MPU6050 imu;
// int16_t ax, ay, az, gx, gy, gz;
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

int vx, vy;
int buttonL = 0; // IO0 button
int buttonR = 4;
int buttonLstate = HIGH; 
int buttonRstate = HIGH; 

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(buttonL, INPUT_PULLUP);
  pinMode(buttonR, INPUT_PULLUP);

  Serial.print("MPU6050 initializing");
  imu.initialize();
  
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  imu.getAcceleration(&accY, &accX, &accZ);

  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accX, accZ) * RAD_TO_DEG;
  double pitch = atan(-accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif

  kalmanY.setAngle(roll); // Set starting angle
  kalmanX.setAngle(pitch);
  gyroYangle = roll;
  gyroXangle = pitch;
  compAngleY = roll;
  compAngleX = pitch;

  timer = micros();

  while (!imu.testConnection()) { Serial.print("."); }
  Serial.println();  
  Serial.println("BLE Mouse starting !");
  bleMouse.begin();
}

void loop() {
   /* Update IMU sensor values */
  imu.getAcceleration(&accY, &accX, &accZ);
  imu.getRotation(&gyroY, &gyroX, &gyroZ);
  
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians)
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accX, accZ) * RAD_TO_DEG;
  double pitch = atan(-accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
#endif

   double gyroXrate = gyroX / 131.0; // Convert to deg/s
   double gyroYrate = gyroY / 131.0; // Convert to deg/s
 
  
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleY > 90) || (roll > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(roll);
    compAngleY = roll;
    kalAngleY = roll;
    gyroYangle = roll;
  } else
    kalAngleY = kalmanY.getAngle(roll, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleX > 90) || (pitch > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(pitch);
    compAngleX = pitch;
    kalAngleX = pitch;
    gyroXangle = pitch;
  } else
    kalAngleX = kalmanX.getAngle(pitch, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

   compAngleX= 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
   compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
    if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
    if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;


  vx =  (compAngleY+4);  
  vy = (compAngleX);
  
  Serial.print("gyroX = ");   Serial.print(gyroX);
  Serial.print(", gyroY = "); Serial.print(gyroY);
  Serial.print("\t");
  Serial.print("X = ");    Serial.print(vx);
  Serial.print(", Y = ");  Serial.println(vy);
  
  bleMouse.move(vx,vy);
    
  buttonLstate = digitalRead(buttonL);
  buttonRstate = digitalRead(buttonR);  
  
  if (buttonLstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_LEFT);
    delay(6);
  } 
  else if(buttonRstate == LOW) { // press button to Ground
    bleMouse.click(MOUSE_RIGHT);
    delay(6);
  }
  delay(6);
}
