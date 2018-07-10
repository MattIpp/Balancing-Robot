#include <Wire.h>

//Time Variables
long lastTimeMillis;
long deltaTimeMillis;
float deltaTime;

//IMU Variables
const int IMUPin = 0x68;
long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;
float accelerationX, accelerationY, accelerationZ;
long gyroX, gyroY, gyroZ;
float rotX, rotY, rotZ;
double angleX, angleY, angleZ;

//PID Variables
float PID = 0;
float prop = 0;
float integ = 0;
float deriv = 0;
float targetAngle = 90;
float kp = 0.08;
float ki = 0.012;
float kd = 0.004;
float k = 2;
float error = 0;
float deltaError = 0;
float lastError = 0;
int motorDirect;

//Stay in place variables
int wheelError = 0;
float PID2 = 0;
float prop2 = 0;
float kp2 = 0.1f;
bool stayInPlace = false;

//Motor Speed variables
int delayTime;
float tps;
int motorTimer = 0;

void setup() 
{
  //Wake from sleep mode
  Wire.begin();
  Wire.beginTransmission(IMUPin);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();

  //Set gyro sensitivity
  Wire.beginTransmission(IMUPin);
  Wire.write(0x1B);
  Wire.write(0b00000000);
  Wire.endTransmission();

  //Set Accelerometer Sensitivity
  Wire.beginTransmission(IMUPin);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(7, OUTPUT);

  //Activate Low Pass Filter
  Wire.beginTransmission(IMUPin);
  Wire.write(0x1A);
  Wire.write(0b00000110);
  Wire.endTransmission();

  Serial.begin(9600);

  delay(500);
  
  readMPU();
  while(gForceZ > 1)
    readMPU();
  angleY = 180.0f / 3.1416f * asin(gForceZ);
  //Serial.println(angleY);
}

void loop() 
{
  //Set Time variables
  deltaTimeMillis = millis() - lastTimeMillis;
  deltaTime = (float)deltaTimeMillis / 1000.0f;
  motorTimer += deltaTimeMillis;
  lastTimeMillis = millis();

  getPID2();
  if(stayInPlace)
    targetAngle = 90 + PID2;

  readMPU();
  getPID();

  //Set motor rate, PID = rotations/sec
  if(true)
  {
    if(PID > 0)
    {
      motorDirect = 0;
      setDirection();
    }
    else
    {
      PID *= -1;
      motorDirect = 1;
      setDirection();
    }
    tps = PID * 200;
    delayTime = 1000 / tps;
  }

  //Check if ping motor
  if(millis() < 5000)
  {
    motorTimer = 0;
    PID = 0;
  }
  if(delayTime > 0 && motorTimer >= delayTime)
  {
    motorTimer = 0;
    pingMotor();
    Serial.println("yay");
  }

  //Serial.println(error);
}

void pingMotor()
{
  digitalWrite(4, HIGH);
  digitalWrite(6, HIGH);
  delayMicroseconds(1);
  digitalWrite(4, LOW);
  digitalWrite(6, LOW);
  delayMicroseconds(1);
  if(motorDirect == 1)
  {
    wheelError++;
  }
  else
  {
    wheelError--;
  }
}

void setDirection()
{
  digitalWrite(5, motorDirect);
  digitalWrite(7, motorDirect);
}

void getPID()
{
  lastError = error;
  error = targetAngle - angleY;
  deltaError = error - lastError;

  //P
  prop = kp * error;
  //I
  integ = ki * error * deltaTime;
  //D
  deriv = kd * deltaError / deltaTime;

  PID = prop + integ + deriv;
  PID *= k;
}

void getPID2()
{
  prop2 = kp2 * wheelError;
  PID2 = prop2;
}

void readMPU()
{
  Wire.beginTransmission(IMUPin);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(IMUPin, 6);
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read();
  accelY = Wire.read()<<8|Wire.read();
  accelZ = Wire.read()<<8|Wire.read();

  gForceX = accelX / 16384.0f;
  gForceY = accelY / 16384.0f;
  gForceZ = accelZ / 16384.0f - 0.1f;

  Wire.beginTransmission(IMUPin);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(IMUPin, 6);
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read();
  gyroY = Wire.read()<<8|Wire.read();
  gyroZ = Wire.read()<<8|Wire.read();

  rotX = (float)gyroX / 131.0f + 1.3;
  rotY = (float)gyroY / 131.0f + 2.55;
  rotZ = (float)gyroZ / 131.0f - 1.65;

  angleX += deltaTime * rotX * 0.9996;
  angleY -= deltaTime * rotY;
  angleZ += deltaTime * rotZ * 0.9996;
}

