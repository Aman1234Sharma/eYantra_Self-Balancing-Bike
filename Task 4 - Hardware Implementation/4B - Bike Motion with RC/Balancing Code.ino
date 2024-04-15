#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <RF24.h>

MPU6050 mpu(Wire);

unsigned long timer = 0;

#define motorPin1 A2
#define enablePin1 6
#define motorPin2 A3

#define boMotorPin1 9   // Replace with your actual BO motor control pin
#define boEnablePin 5   // Replace with your actual BO motor enable pin
#define boMotorPin2 4    // Replace with your actual BO motor control pin

#define NRF_CE 10
#define NRF_CSN 8

RF24 radio(NRF_CE, NRF_CSN);
const byte address = 5;

// Roll PID parameters
double Kp_roll = 50;    // Proportional gain
double Ki_roll = 0;     // Integral gain
double Kd_roll = 10;    // Derivative gain

// Yaw PID parameters
double Kp_yaw = 0;     // Proportional gain for yaw control
double Ki_yaw = 0;      // Integral gain for yaw control
double Kd_yaw = 0 ;     // Derivative gain for yaw control

// Target angle for self-balancing
double target_roll = 0;
double target_yaw = 0;

// PID variables for roll
double previousError_roll = 0;
double integral_roll = 0;

// PID variables for yaw
double previousError_yaw = 0;
double integral_yaw = 0;

// Motor control variables
int motorSpeed = 0;
int boMotorSpeed = 0;

void setup() {
  Serial.begin(9600);

  Wire.begin(); // Initialize the Wire library
  mpu.begin(); // Initialize the MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");

  // Initialize motor control pins for the main motor
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  // Initialize motor control pins for the BO motor
  pinMode(boMotorPin1, OUTPUT);
  pinMode(boMotorPin2, OUTPUT);
  pinMode(boEnablePin, OUTPUT);

  // Initialize NRF24
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MIN);
}

void executeCommand(char command) {
  switch (command) {
    case 'F':
      Serial.println("*");
      digitalWrite(boMotorPin1, HIGH);
      digitalWrite(boMotorPin2, LOW);
      break;
    case 'B':
      digitalWrite(boMotorPin1, LOW);
      digitalWrite(boMotorPin2, HIGH);
      break;
    case 'L':
//      digitalWrite(boMotorPin1, LOW);
//      digitalWrite(boMotorPin2, LOW);
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
      break;
    case 'R':
//      digitalWrite(boMotorPin1, LOW);
//      digitalWrite(boMotorPin2, LOW);
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
      break;
    case 'X':
      digitalWrite(boMotorPin1, LOW);
      digitalWrite(boMotorPin2, LOW);
      break;
  }
}

void loop() {
  mpu.update();

  double roll = mpu.getAngleX();
  double yaw = mpu.getAngleZ(); // Assuming Z-axis is for yaw control

  // Calculate roll PID error
  double error_roll = target_roll - roll;
  integral_roll += error_roll;
  double derivative_roll = error_roll - previousError_roll;
  double output_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll;
    // Calculate yaw PID error
  double error_yaw = target_yaw - yaw;
  integral_yaw += error_yaw;
  double derivative_yaw = error_yaw - previousError_yaw;
  double output_yaw = Kp_yaw * error_yaw + Ki_yaw * integral_yaw + Kd_yaw * derivative_yaw;
  double total_error = output_roll - output_yaw;

  // Map PID output to motor speed for roll control
  motorSpeed = map(total_error, -90, 90, -255, 255);
  motorSpeed = constrain(motorSpeed, -255, 255);

  // Motor control based on PID output for roll control
  if (motorSpeed > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
  analogWrite(enablePin1, abs(motorSpeed));




  // Update previous errors for both roll and yaw
  previousError_roll = error_roll;
  previousError_yaw = error_yaw;

  // NRF Communication
  radio.startListening();
  delay(10);
  if (radio.available()) {
    char command = 'S'; 
    radio.read(&command, sizeof(command));
    analogWrite(boEnablePin,180);
    analogWrite(enablePin1,100);
    executeCommand(command);
  }

  delay(10);
}
