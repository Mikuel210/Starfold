/*


#include "mpu9250.h"
#include "SensorFusion.h"
#include <Arduino.h>    
#include <Wire.h>       
#include <TFLI2C.h>
#include <ESP32Servo.h>

// STATE INDICATION
#define LED_R 2
#define LED_G 19
#define LED_B 23
#define BUZZER 27

// BMS
#define BMS_1 36

// ACTUATORS
#define LEG 4
Servo Leg;

// DISTANCE
TFLI2C tflI2C;

int16_t tfDist; 
int16_t tfAddr = TFL_DEF_ADR; 

// ORRIENTATION
bfs::Mpu9250 imu;
SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch = 0;
float roll = -180;
float yaw = 240;
float deltat;

float gxOffset;
float gyOffset;
float gzOffset;
                               

void setup()
{
  Serial.begin(115200);
  while(!Serial) {}

  Wire.begin();
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);

  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  // PINMODE
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  // ACTUATORS
  Leg.attach(LEG);

  // CALIBRATION
  int samples = 100;

  for (int i = 0; i < samples; i++)
  {
    if (imu.Read())
    {
      imu.new_imu_data();
      imu.new_mag_data();
      gxOffset += imu.gyro_x_radps();
      gyOffset += imu.gyro_y_radps();
      gzOffset += imu.gyro_z_radps();
    }

    delay(50);
  }

  gxOffset /= samples;
  gyOffset /= samples;
  gzOffset /= samples;
}

void plot(float number){
  Serial.print(number); Serial.print(",");
}

void setLimits(float lowerEnd, float upperEnd)
{
  plot(lowerEnd);
  plot(upperEnd);
}

void endPlot(){
  Serial.println();
}

void loop()
{
  setLimits(-180, 280);
  unsigned long usStart = micros();

  if (tflI2C.getData(tfDist, tfAddr))
    plot(tfDist);

  else tflI2C.printStatus();

  if (imu.Read()) {
    imu.new_imu_data();
    imu.new_mag_data();
    
    ax = imu.accel_x_mps2();
    ay = imu.accel_y_mps2();
    az = imu.accel_z_mps2();
    gx = imu.gyro_x_radps() - gxOffset;
    gy = imu.gyro_y_radps() - gyOffset;
    gz = imu.gyro_z_radps() - gzOffset;
    mx = imu.mag_x_ut();
    my = imu.mag_y_ut();
    mz = imu.mag_z_ut();

    deltat = fusion.deltatUpdate();
    fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);

    pitch = fusion.getPitch();
    roll = fusion.getRoll();    //you could also use getRollRadians() ecc
    yaw = fusion.getYaw();
    
    plot(pitch);
    plot(roll);
    plot(yaw);
  }

  plot(analogRead(BMS_1) / 4095.0 * 3.3 * 2);
  endPlot();

  int ledState = (millis() / 1000) % 3;

  if (ledState == 0) {
    digitalWrite(LED_R, HIGH);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, LOW);
  } else if (ledState == 1) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, HIGH);
    digitalWrite(LED_B, LOW);
  } else if (ledState == 2) {
    digitalWrite(LED_R, LOW);
    digitalWrite(LED_G, LOW);
    digitalWrite(LED_B, HIGH);
  }

  int legState = (millis() / 1500) % 2;

  if (legState == 0) {
    Leg.write(0);
  } else {
    Leg.write(180);
  }

  unsigned long usEnd = micros();
  delayMicroseconds(20000 - (usEnd - usStart));
}


*/