#include <SoftwareSerial.h>
#include <Servo.h>
#include "hcsr04.h"

// motor pins
#define leftMotorsForward 40
#define leftMotorsBackward 41
#define leftMotorPWM 8

#define rightMotorsForward 52
#define rightMotorsBackward 53
#define rightMotorPWM 9

// parameters
unsigned short motorSpeed = 100;
unsigned short minMotorSpeed = 140;  // min. 0
unsigned short maxMotorSpeed = 200;  // max. 255
unsigned short motorSpeedForTurnsInAutomaticMode = 200;
unsigned short SPEED_INCREMENT = 20;

char c = ' ';
bool NL = true;

char Incoming_value[3] = "";

bool driveAutonome = false;

bool start = false;
bool pause = false;

bool forward = false;
bool backward = false;
bool left = false;
bool right = false;

bool xManuellDrive = false;
bool speedDown = false;
bool speedUp = false;

bool SERVO_TURN_IN_AUTOMATIC_MODE = true;
const int delayTime = 300;
const int crashDistance = 700;  // 500
const int manuellCrashDistance = 500;

const int redLED = 13;
const int blueLED = 12;

Servo sensorServo;

unsigned long previousMillisServo = 0;
unsigned long previousMillisMotor = 0;
unsigned long previousMillisDrive = 0;
const long intervalServo = 75;
const long intervalMotor = 600;
const long intervalDrive = 500;

void setup() {
  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  Serial1.begin(9600);

  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  sensorServo.attach(4);

  digitalWrite(trig, HIGH);

  Serial.begin(9600);

  pinMode(rightMotorPWM, OUTPUT);
  pinMode(rightMotorsForward, OUTPUT);
  pinMode(rightMotorsBackward, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(leftMotorsForward, OUTPUT);
  pinMode(leftMotorsBackward, OUTPUT);

  analogWrite(leftMotorPWM, motorSpeed);
  analogWrite(rightMotorPWM, motorSpeed);

  sensorServo.write(90);
}

void setAllBooleansForControlToFalse();
void checkBluetoothCommands();
void autonomicDrive();

void loop() {
  unsigned long currentMillis = millis();

  checkBluetoothCommands();
  redLEDOn();

  while (driveAutonome) {
    autonomicDrive();
    blueLEDOn();
    redLEDOn();
    checkBluetoothCommands();
  }

  while (right) {
    initServo();
    redLEDOff();
    blueLEDOn();
    turnRight();
    checkBluetoothCommands();
  }

  while (left) {
    initServo();
    redLEDOff();
    blueLEDOn();
    turnLeft();
    checkBluetoothCommands();
  }

  while (backward) {
    initServo();
    redLEDOff();
    blueLEDOn();
    driveBackwards();
    checkBluetoothCommands();
  }

  while (forward) {
    initServo();
    redLEDOff();
    blueLEDOn();
    driveForward();
    checkBluetoothCommands();
  }

  while (xManuellDrive) {
    initServo();
    redLEDOff();
    blueLEDOn();
    checkBluetoothCommands();
  }

  while (pause) {
    initServo();
    blueLEDOff();
    redLEDOn();
    checkBluetoothCommands();
  }

  delay(10);
}

void checkBluetoothCommands() {
  if (Serial1.available() > 0) {
    int length = Serial1.readBytes(Incoming_value, 2);
    Incoming_value[length] = '\0';

    Serial.print(Incoming_value);

    if (strcmp(Incoming_value, "A0") == 0) {  // Start again autonome drive after pause
      setAllBooleansForControlToFalse();
      driveAutonome = true;
    } else if (strcmp(Incoming_value, "P0") == 0) {  // Pause
      setAllBooleansForControlToFalse();
      stopMotors();
      pause = true;
    } else if (strcmp(Incoming_value, "F0") == 0) {  // Forwards
      setAllBooleansForControlToFalse();
      forward = true;
    } else if (strcmp(Incoming_value, "B0") == 0) {  // Backwards
      setAllBooleansForControlToFalse();
      backward = true;
    } else if (strcmp(Incoming_value, "L0") == 0) {  // Left turn
      setAllBooleansForControlToFalse();
      left = true;
    } else if (strcmp(Incoming_value, "R0") == 0) {  // Right turn manually
      setAllBooleansForControlToFalse();
      right = true;
    } else if (strcmp(Incoming_value, "X0") == 0) {  // Manual drive
      setAllBooleansForControlToFalse();
      stopMotors();
      xManuellDrive = true;
    } else if (strcmp(Incoming_value, "S0") == 0) {  // Speed down
      decreaseSpeed();
    } else if (strcmp(Incoming_value, "C0") == 0) {  // Speed up
      increaseSpeed();
    } else if (strcmp(Incoming_value, "T0") == 0) {  // Autonomic drive
      setAllBooleansForControlToFalse();
      driveAutonome = true;
    }
  }
}

void initServo() {
  if (!driveAutonome) {
    if (sensorServo.read() != 90) {
      sensorServo.write(90);
    }
  }
}

void setAllBooleansForControlToFalse() {
  driveAutonome = false;
  start = false;
  pause = false;
  forward = false;
  backward = false;
  left = false;
  right = false;
  xManuellDrive = false;
  speedDown = false;
  speedUp = false;
}

void autonomicDrive() {
  unsigned long currentMillis = millis();
  int currentDistance = getDistance();

  if (currentDistance > crashDistance) {
    analogWrite(leftMotorPWM, motorSpeed);
    analogWrite(rightMotorPWM, motorSpeed);
    driveForward();

    if (SERVO_TURN_IN_AUTOMATIC_MODE) {
      for (int angle = 50; angle <= 130; angle += 40) {
        sensorServo.write(angle);
        delay(delayTime);
        int distance = getDistance();
        if (distance < crashDistance) {
          stopMotors();
          delay(500);
          driveBackwards();
          delay(250);
          stopMotors();

          // sensor right
          sensorServo.write(0);
          delay(800);
          int distanceRight = getDistance();

          // sensor left
          sensorServo.write(180);
          delay(1000);
          int distanceLeft = getDistance();
          delay(50);
          sensorServo.write(90);

          if (distanceRight >= distanceLeft) {
            turnRight();
          } else {
            turnLeft();
          }
          break;
        }
      }
      sensorServo.write(90);
    }

  } else {
    stopMotors();
    delay(500);
    driveBackwards();
    delay(250);
    stopMotors();

    // sensor right
    sensorServo.write(0);
    delay(800);
    int distanceRight = getDistance();

    // sensor left
    sensorServo.write(180);
    delay(1000);
    int distanceLeft = getDistance();
    delay(50);
    sensorServo.write(90);

    if (distanceRight >= distanceLeft) {
      turnRight();
    } else {
      turnLeft();
    }

    driveForward();
  }
}


void increaseSpeed() {
  motorSpeed += SPEED_INCREMENT;

  if (motorSpeed > maxMotorSpeed) {
    motorSpeed = maxMotorSpeed;
  }

  analogWrite(leftMotorPWM, motorSpeed);
  analogWrite(rightMotorPWM, motorSpeed);

  Serial.print("Speed up to: ");
  Serial.println(motorSpeed);
}

void decreaseSpeed() {
  motorSpeed -= SPEED_INCREMENT;

  if (motorSpeed < minMotorSpeed) {
    motorSpeed = minMotorSpeed;
  }

  analogWrite(leftMotorPWM, motorSpeed);
  analogWrite(rightMotorPWM, motorSpeed);

  Serial.print("Speed down to: ");
  Serial.println(motorSpeed);
}

void stopMotors() {
  digitalWrite(rightMotorsForward, LOW);
  digitalWrite(leftMotorsForward, LOW);
  digitalWrite(rightMotorsBackward, LOW);
  digitalWrite(leftMotorsBackward, LOW);
}

void driveForward() {
  if (!driveAutonome) {
    int currentDistance = getDistance();
    if (currentDistance < manuellCrashDistance) {
      stopMotors();
    }
  }

  digitalWrite(rightMotorsForward, HIGH);
  digitalWrite(leftMotorsForward, HIGH);
  digitalWrite(rightMotorsBackward, LOW);
  digitalWrite(leftMotorsBackward, LOW);
}

void driveBackwards() {
  if (driveAutonome) {
    analogWrite(leftMotorPWM, motorSpeedForTurnsInAutomaticMode);
    analogWrite(rightMotorPWM, motorSpeedForTurnsInAutomaticMode);
  }

  digitalWrite(rightMotorsForward, LOW);
  digitalWrite(leftMotorsForward, LOW);
  digitalWrite(rightMotorsBackward, HIGH);
  digitalWrite(leftMotorsBackward, HIGH);
}

void turnRight() {
  if (right) {
    analogWrite(leftMotorPWM, motorSpeed);
    analogWrite(rightMotorPWM, motorSpeed);
  } else {
    analogWrite(leftMotorPWM, motorSpeedForTurnsInAutomaticMode);
    analogWrite(rightMotorPWM, motorSpeedForTurnsInAutomaticMode);
  }

  digitalWrite(rightMotorsForward, LOW);
  digitalWrite(leftMotorsForward, HIGH);
  digitalWrite(rightMotorsBackward, HIGH);
  digitalWrite(leftMotorsBackward, LOW);

  if (driveAutonome) {
    delay(600);
    stopMotors();
    analogWrite(leftMotorPWM, motorSpeed);
    analogWrite(rightMotorPWM, motorSpeed);
  }
}

void turnLeft() {
  if (left) {
    analogWrite(leftMotorPWM, motorSpeed);
    analogWrite(rightMotorPWM, motorSpeed);
  } else {
    analogWrite(leftMotorPWM, motorSpeedForTurnsInAutomaticMode);
    analogWrite(rightMotorPWM, motorSpeedForTurnsInAutomaticMode);
  }

  digitalWrite(rightMotorsForward, HIGH);
  digitalWrite(leftMotorsForward, LOW);
  digitalWrite(rightMotorsBackward, LOW);
  digitalWrite(leftMotorsBackward, HIGH);

  if (driveAutonome) {
    delay(600);
    stopMotors();
    analogWrite(leftMotorPWM, motorSpeed);
    analogWrite(rightMotorPWM, motorSpeed);
  }
}

void blueLEDOn() {
  digitalWrite(blueLED, HIGH);
}

void blueLEDOff() {
  digitalWrite(blueLED, LOW);
}

void redLEDOn() {
  digitalWrite(redLED, HIGH);
}

void redLEDOff() {
  digitalWrite(redLED, LOW);
}
