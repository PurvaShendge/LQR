#include <Wire.h>
#include <MPU6050.h>
#include "variables.h"
MPU6050 mpu;

float angle = 0;                // pitch angle
float angular_velocity = 0;     // pitch rate
float linear_velocity = 0;      // linear velocity
float Position = 0;             // position
float k[1][4]={{ -1.0000, -1.8811, 27.2561, 3.1007}};
float u=0;
float dt = 0.01; 
# define dirleftA 7
# define dirleftB 8
#define pwmleft 6
# define dirrightA 9
# define dirrightB 10
#define pwmright 11


void setup() {
  pinMode(dirleftA, OUTPUT);
  pinMode(dirleftB, OUTPUT);
  pinMode(pwmleft, OUTPUT);
  pinMode(dirrightA, OUTPUT);
  pinMode(dirrightB, OUTPUT);
  pinMode(pwmright, OUTPUT);
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();

  // set MPU6050 to full-scale range of ±250 degrees/s
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // calibrate the MPU6050 gyro
  mpu.CalibrateGyro();

  // set the MPU6050 to sleep mode to save power
 // mpu.setSleepEnabled(true);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // convert raw gyro readings to degrees per second
  float gyro_x = (float)gx / 131.0;

  // update the pitch angle and rate using the gyro readings
  angle = angle + (gyro_x * dt);
  angular_velocity = gyro_x;

  // calculate the linear velocity using the trapezoidal rule
  linear_velocity = linear_velocity + ((angular_velocity + gyro_x) / 2.0 * dt);

  // integrate the linear velocity to obtain the position
  Position = Position + linear_velocity * dt;
  u = -(k[0][0] *Position + k[0][1] *linear_velocity + k[0][2]*angle + k[0][3]*angular_velocity);
  u = constrain(u, -150, 150);
//  Serial.print(u);
//  Serial.print("  ");
//  Serial.print(omega);
//  Serial.print("  ");
//  Serial.println();
  if (u > 0) {
    digitalWrite(dirleftA, HIGH);
    digitalWrite(dirleftB, LOW);
    digitalWrite(dirrightA, LOW);
    digitalWrite(dirrightB, HIGH);
  }
  else {
    digitalWrite(dirleftA, LOW);
    digitalWrite(dirleftB, HIGH);
    digitalWrite(dirrightA, HIGH);
    digitalWrite(dirrightB, LOW);
  }
  analogWrite(pwmleft, abs(u));
  analogWrite(pwmright,abs(u));
}
