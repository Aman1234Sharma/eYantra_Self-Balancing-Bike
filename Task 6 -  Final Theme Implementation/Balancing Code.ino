//////////////////////////////////////////////////////// LIBRARY IMPORTS ////////////////////////////////////////////////////////////////////
#include <Wire.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <RF24.h>

//////////////////////////////////////////////////////// PIN DEFINATIONS ////////////////////////////////////////////////////////////////////

MPU6050 mpu(Wire);

unsigned long timer = 0;

#define buzzerPin 7
#define greenLedPin A1
#define redLedPin A0
#define hallSensorPin A7

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
///////////////////////////////////////////////////////////// PID TUNING ///////////////////////////////////////////////////////////////////
// PID parameters
double Kp = 20;    // Proportional gain //20
double Ki = 0;     // Integral gain  //0
double Kd = 50 ;    // Derivative gain  //50

// PID parameters for yaw
double Kp_yaw = 2;  // Proportional gain for yaw
double Ki_yaw = 0;  // Integral gain for yaw
double Kd_yaw = 50; // Derivative gain for yaw 

///////////////////////////////////////////////////////// VARIABLE INITIALISTION /////////////////////////////////////////////////////////////////////////
double targetroll = 0;
double targetYaw = 0; 

double previousError = 0;
double previousYawError = 0;
double integral = 0;
double integralYaw = 0;

// Motor control variables
int motorSpeed = 0;
int boMotorSpeed = 0;

void setup() {
  Serial.begin(9600);

  Wire.begin(); // Initialize the Wire library
  mpu.begin();  // Initialize the MPU6050

  mpu.calcOffsets(); // gyro and accelero

  // Initialize motor control pins for the main motor
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  // Initialize motor control pins for the BO motor
  pinMode(boMotorPin1, OUTPUT);
  pinMode(boMotorPin2, OUTPUT);
  pinMode(boEnablePin, OUTPUT);

  pinMode(buzzerPin,OUTPUT);
  pinMode(greenLedPin,OUTPUT);
  pinMode(redLedPin,OUTPUT);
  pinMode(hallSensorPin,INPUT );
  

  // Initialize NRF24
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(RF24_PA_MIN);
}

void executeCommand(char command) {
  switch (command) {
    // 1 Forward command
    case 'F':
      digitalWrite(boMotorPin1, HIGH);
      digitalWrite(boMotorPin2, LOW);
      break;
    case 'B':
      // Backward command
      digitalWrite(boMotorPin1, LOW);
      digitalWrite(boMotorPin2, HIGH);
      break;
    case 'L':
      targetYaw += 5 ; 
//      //delay(5);
      break;
    case 'R':
      targetYaw -= 5; 
//      //delay(5);
      break;
    case 'S':
      digitalWrite(boMotorPin1, LOW);
      digitalWrite(boMotorPin2, LOW);
      break;
    case 'X':
    int sensorValue = analogRead(hallSensorPin);

    // Determine the polarity based on the analog value
  if (sensorValue < 410) {
    

    // Beep twice with a 1-second delay
    for (int i = 0; i < 2; i++) {
      digitalWrite(buzzerPin, HIGH);
      delay(1000);
      digitalWrite(buzzerPin, LOW);
      delay(1000);
    }

    // Turn on the red LED for 3 seconds
    digitalWrite(redLedPin, HIGH);
    delay(3000);
    digitalWrite(redLedPin, LOW);
  } else if(sensorValue >410 and sensorValue<520){
    
    // Beep twice with a 1-second delay
    for (int i = 0; i < 2; i++) {
      digitalWrite(buzzerPin, HIGH);
      delay(1000);
      digitalWrite(buzzerPin, LOW);
      delay(1000);
    }

    // Turn on the green LED for 3 seconds
    digitalWrite(greenLedPin, HIGH);
    delay(3000);
    digitalWrite(greenLedPin, LOW);
  }

  }
}

void pid(){
  mpu.update();
  double roll = mpu.getAngleX();
  double yaw = mpu.getAngleZ(); // Assuming Z axis corresponds to Yaw
   
  // Calculate Roll PID error
  double rollError = targetroll - roll;

  // Calculate Yaw PID error
  double yawError = targetYaw - yaw;

  // Update integral and derivative terms for Roll
  integral += rollError;
  double derivative = rollError - previousError;

  // Update integral and derivative terms for Yaw
  integralYaw += yawError;
  double derivativeYaw = yawError - previousYawError;

  // Calculate Roll PID output
  double output = Kp * rollError + Ki * integral + Kd * derivative;

  // Calculate Yaw PID output
  double yawOutput = Kp_yaw * yawError + Ki_yaw * integralYaw + Kd_yaw * derivativeYaw;

  // Combine Roll and Yaw PID outputs (adjust weights as needed)
  double combinedOutput = output + yawOutput;

  // Map combined PID output to motor speed
  motorSpeed = map(combinedOutput, -90, 90, -255, 255);

  // Constrain motor speed
  motorSpeed = constrain(motorSpeed, -255, 255);

  // Motor control based on combined PID output for the main motor
  if (motorSpeed > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }

  // Set motor speed using PWM for the main motor
  analogWrite(enablePin1, abs(motorSpeed));

  // Update previous errors for Roll and Yaw
  previousError = rollError;
  previousYawError = yawError;

}
void loop() {
  
  // Read MPU6050 sensor data
  pid();
  // NRF Communication
  radio.startListening();
  delay(10);
  if (radio.available()) {
    char command = 'S';
    radio.read(&command, sizeof(command));
    
    analogWrite(boEnablePin, 200);
    executeCommand(command);
   
  }

}
