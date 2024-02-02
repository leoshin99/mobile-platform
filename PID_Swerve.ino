#include <ESP32Servo.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// PID Constants
float kp = 5;
float ki = 6;
float kd = 0;

// Variables for PID control
float e1integral = 0;
float e2integral = 0;
float u1;
float u2;

// Encoder variables
volatile long pos1_i = 0;
volatile long pos2_i = 0;
volatile long pos1Prev = 0;
volatile long pos2Prev = 0;
volatile float rpm1Filt = 0;
volatile float rpm2Filt = 0;
volatile float rpm1Prev = 0;
volatile float rpm2Prev = 0;
volatile long currT = 0;
volatile long prevT = 0;
float prevError1 = 0; // Added for PID function
float prevError2 = 0;

float ppr = 715;

const int ENC_1A = 4;
const int ENC_1B = 5;
const int ENC_2A = 32;
const int ENC_2B = 33;

//********DCMotor***********
const int MOTOR_1A =18;
const int MOTOR_1B =19;
const int PWM_1 =15;

const int MOTOR_2A =26;
const int MOTOR_2B =27;
const int PWM_2 =25;

const int stby = 13;

const int CHANNEL_1 =2;
const int CHANNEL_2 =3;

const int MOTOR_FREQ = 6000;
const int MOTOR_RESOLUTION =10;

//********Servo***********
Servo servo1;
Servo servo2;

const int servo1Pin = 16;
const int servo2Pin = 17;

// float cor;
float Servo1Input;
float Servo2Input;

int servo1Offset = 1550;
int servo2Offset = 1510;

float angle;
float angle_s;
int Servo1Pwm;
int Servo1Pwm_s;
int Servo2Pwm;
int Servo2Pwm_s;



void setup() {
  
  pinMode(ENC_1A, INPUT_PULLUP);
  pinMode(ENC_1B, INPUT_PULLUP);
  pinMode(ENC_2A, INPUT_PULLUP);
  pinMode(ENC_2B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_1A), Interrupt_1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_1B), Interrupt_1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_2A), Interrupt_2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_2B), Interrupt_2B, CHANGE);
  
  SerialBT.begin("ESP");
  Serial.begin(115200);

  //********Servo***********
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  servo1.writeMicroseconds(servo1Offset);
  servo2.writeMicroseconds(servo2Offset);

  //********DCMotor***********

  pinMode(MOTOR_1A, OUTPUT);
  pinMode(MOTOR_1B, OUTPUT);
  pinMode(MOTOR_2A, OUTPUT);
  pinMode(MOTOR_2B, OUTPUT);
  pinMode(stby, OUTPUT);

	ledcAttachPin(PWM_1, CHANNEL_1);
	ledcAttachPin(PWM_2, CHANNEL_2);

	ledcSetup(CHANNEL_1, MOTOR_FREQ, MOTOR_RESOLUTION);
	ledcSetup(CHANNEL_2, MOTOR_FREQ, MOTOR_RESOLUTION);   

  digitalWrite(stby, HIGH);

  driveMotor(MOTOR_1A, MOTOR_1B, CHANNEL_1, 0);
  driveMotor(MOTOR_2A, MOTOR_2B, CHANNEL_2, 0);

	delay(5000);
}

void loop() {
  // Read joystick values
  if(SerialBT.available()>0) {
    byte receivedData[6];
    SerialBT.readBytes(receivedData, 6);
    
    int x = (receivedData[1] << 8) | receivedData[0];
    int y = (receivedData[3] << 8) | receivedData[2];
    int x_s = (receivedData[5] << 8) | receivedData[4];

    Servo1Pwm = map(x,0,1800,-650,635);
    Servo2Pwm = map(x,0,1800,-650,635);

    Servo1Pwm_s = map(x_s,0,1800,-650,635);
    Servo2Pwm_s = map(x_s,0,1800,-650,635);

    // angle = map(ServoPwm,-950,950,-90,90);
    // angle_s = map(ServoPwm_s,-950,950,-90,90);

    Servo1Input =  servo1Offset - Servo1Pwm - Servo1Pwm_s;
    Servo2Input =  servo2Offset + Servo2Pwm - Servo2Pwm_s;
    servo1.writeMicroseconds(Servo1Input); 
    servo2.writeMicroseconds(Servo2Input);     

    // Serial.print(x,2);
    // Serial.print(" ");
    // Serial.print(ServoPwm);
    // Serial.print(" ");
    // Serial.print(angle);
    // Serial.print(" ");
    // Serial.print(Servo1Input);
    // Serial.print(" ");
    // Serial.print(Servo2Input);
    // // Serial.print(" ");
    // // Serial.print();
    // // Serial.print(" ");
    // // Serial.print();
    // // Serial.print(" ");
    // Serial.println();

    // ***************DC Motor***********************
    // Convert y to target speed (adjust scale factor as needed)
    float rpmTarget = map(y, 0, 600, -300, 300);

    if (rpmTarget <= 5 && rpmTarget >= -5){
      driveMotor(MOTOR_1A, MOTOR_1B, CHANNEL_1, 0);
      driveMotor(MOTOR_2A, MOTOR_2B, CHANNEL_2, 0);
    }

    else{

    
    
      int pos1 = 0;
      int pos2 = 0;
      noInterrupts(); // disable interrupts temporarily while reading
      pos1 = pos1_i;
      pos2 = pos2_i;
      interrupts(); // turn interrupts back on

      // Compute velocity
      currT = micros();
      float deltaT = ((float) (currT-prevT))/1.0e6;
      float velocity1 = (pos1 - pos1Prev)/deltaT;
      float velocity2 = (pos2 - pos2Prev)/deltaT;
      pos1Prev = pos1;
      pos2Prev = pos2;
      prevT = currT;
      delayMicroseconds(1);

      // Compute velocity
      float rpm1 = velocity1 / ppr * 60.0;
      float rpm2 = velocity2 / ppr * 60.0;

      // Low-pass filter (25 Hz cutoff)
      rpm1Filt = 0.854 * rpm1Filt + 0.0728 * rpm1 + 0.0728 * rpm1Prev;
      rpm2Filt = 0.854 * rpm2Filt + 0.0728 * rpm2 + 0.0728 * rpm2Prev;

      rpm1Prev = rpm1;
      rpm2Prev = rpm2;

      // PID calculation
      calculatePID(e1integral, prevError1, rpmTarget, rpm1Filt, u1, kp, ki, kd, deltaT);
      calculatePID(e2integral, prevError2, -1*rpmTarget, rpm2Filt, u2, kp, ki, kd, deltaT);


      // Clip PID output to the valid PWM range
      u1 = constrain(u1, -1023, 1023);
      u2 = constrain(u2, -1023, 1023);


      // Drive motors based on PID output
      driveMotor(MOTOR_1A, MOTOR_1B, CHANNEL_1, u1);
      driveMotor(MOTOR_2A, MOTOR_2B, CHANNEL_2, u2);
    }

    Serial.print(rpmTarget);
    Serial.print(" ");
    Serial.print(-1*rpmTarget);
    Serial.print(" ");
    Serial.print(rpm1Filt);
    Serial.print(" ");
    Serial.print(rpm2Filt);
    Serial.print(" ");
    // Serial.println();

    Serial.print(static_cast<int>(u1));   
    Serial.print(" ");
    Serial.print(static_cast<int>(u2));   
    Serial.print(" ");
    Serial.print(1023);  
    Serial.print(" ");
    Serial.print(-1023);  
    Serial.println();
  }
}

void A_encoderInterrupt(int A, int B, volatile long &position) {
  if (A == 1 && B == 0) {
    position++;
  } else if (A == 1 && B == 1) {
    position--;
  } else if (A == 0 && B == 1) {
    position++;
  } else {
    position--;
  }
}


void B_encoderInterrupt(int A, int B, volatile long &position) {
  if (A == 1 && B == 1) {
    position++;
  } else if (A == 0 && B == 1) {
    position--;
  } else if (A == 0 && B == 1) {
    position++;
  } else {
    position--;
  }
}

void Interrupt_1A() {
  int A = digitalRead(ENC_1A);
  int B = digitalRead(ENC_1B);
  A_encoderInterrupt(A, B, pos1_i);
}

void Interrupt_1B() {
  int A = digitalRead(ENC_1A);
  int B = digitalRead(ENC_1B);
  B_encoderInterrupt(A, B, pos1_i);
}

void Interrupt_2A() {
  int A = digitalRead(ENC_2A);
  int B = digitalRead(ENC_2B);
  A_encoderInterrupt(A, B, pos2_i);
}

void Interrupt_2B() {
  int A = digitalRead(ENC_2A);
  int B = digitalRead(ENC_2B);
  B_encoderInterrupt(A, B, pos2_i);
}


void calculatePID(float &integral, float &prevError, float targetRPM, float measuredRPM, float &output, float kp, float ki, float kd, float deltaT) {
    float error = targetRPM - measuredRPM;
    integral += error * deltaT;
    float derivative = (error - prevError) / deltaT;
    output = kp * error + ki * integral + kd * derivative;
    prevError = error;
}

void driveMotor(int motorA, int motorB, int channel, int pwmValue) {

  pwmValue = static_cast<int>(pwmValue);

  if (pwmValue == 0) { // Brake
  
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, LOW);
	ledcWrite(channel, 0);

  } else if (pwmValue > 0) { // Forward
    if (abs(pwmValue) > 1023){
    pwmValue = 1023;
  }
  digitalWrite(motorA, HIGH);
  digitalWrite(motorB, LOW);
	ledcWrite(channel, pwmValue);;


  } else { // Reverse
   if (abs(pwmValue) > 1023){
    pwmValue = -1023;
  }
  digitalWrite(motorA, LOW);
  digitalWrite(motorB, HIGH);
  ledcWrite(channel, -1*pwmValue);

  }
}