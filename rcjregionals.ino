int motorLeftPin = 9;
int motorRightPin = 11;

int motorBackLeftPin = 12;
int motorBackRightPin = 11;

bool left_color, right_color;

int seconds = 0;
int distCm = 0;

const int lS0 = 2;
const int lS1 = 3;
const int lS2 = 4;
const int lS3 = 5;
const int lOUT = 6;

const int rS0 = 22;
const int rS1 = 23;
const int rS2 = 24;
const int rS3 = 25;
const int rOUT = 26;

int lredValue = 0;
int lgreenValue = 0;
int lblueValue = 0;

int rredValue = 0;
int rgreenValue = 0;
int rblueValue = 0;

int sensorPins[] = {A0, A1, A2, A3, A4};  

float Kp = 22.0; 
float Ki = 0.0;   
float Kd = 2.0;  
float lastError = 0.0;
float integral = 0.0;

int baseSpeed = 40;
int maxSpeed = 255;

int blackThreshold = 600;
int whiteThreshold = 300;

int readSensors() {
  int sensorVals[5];
  int position = 0;
  int weightSum = 0;
  int valueSum = 0;

  for (int i = 0; i < 5; i++) {
    sensorVals[i] = analogRead(sensorPins[i]);
    
    if (sensorVals[i] > blackThreshold) {
      sensorVals[i] = 1000;
    } else if (sensorVals[i] < whiteThreshold) {
      sensorVals[i] = 0;
    }
    
    position += sensorVals[i] * i;
    valueSum += sensorVals[i];
  }

  if (valueSum == 0) {
    return lastError;
  }

  return position / valueSum - 2; 
}

void calculatePID() {
  int error = readSensors();
  integral += error;
  float derivative = error - lastError;

  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  int leftMotorSpeed = baseSpeed - correction;
  int rightMotorSpeed = baseSpeed + correction;

  leftMotorSpeed = constrain(leftMotorSpeed, -maxSpeed, maxSpeed);
  rightMotorSpeed = constrain(rightMotorSpeed, -maxSpeed, maxSpeed);

  Serial.print("Left Motor: ");
  Serial.print(leftMotorSpeed);
  Serial.print(" Right Motor: ");
  Serial.println(rightMotorSpeed);

  analogWrite(motorLeftPin, abs(leftMotorSpeed));
  analogWrite(motorRightPin, abs(rightMotorSpeed));
}

void forward(){
  analogWrite(motorBackRightPin, 0);
  analogWrite(motorLeftPin, 50);
  analogWrite(motorRightPin, 50);
  analogWrite(motorBackLeftPin, 0);
  delay(900);
}

void left(){
  analogWrite(motorBackRightPin, 0);
  analogWrite(motorLeftPin, 0);
  analogWrite(motorRightPin, 70);
  analogWrite(motorBackLeftPin, 20);
  delay(1250);
}

void right(){
  analogWrite(motorBackRightPin, 20);
  analogWrite(motorLeftPin, 70);
  analogWrite(motorRightPin, 0);
  analogWrite(motorBackLeftPin, 0);
  delay(1150);
}

void uturn(){
  analogWrite(motorBackRightPin, 50);
  analogWrite(motorLeftPin, 50);
  analogWrite(motorRightPin, 0);
  analogWrite(motorBackLeftPin, 0);
  delay(2200);
}

void right2(){
  analogWrite(motorBackRightPin, 50);
  analogWrite(motorLeftPin, 50);
  analogWrite(motorRightPin, 0);
  analogWrite(motorBackLeftPin, 0);
  delay(1100);
}

void left2(){
  analogWrite(motorBackRightPin, 50);
  analogWrite(motorLeftPin, 50);
  analogWrite(motorRightPin, 0);
  analogWrite(motorBackLeftPin, 0);
  delay(1100);
}

void stop(){
  analogWrite(motorBackRightPin, 0);
  analogWrite(motorLeftPin, 0);
  analogWrite(motorRightPin, 0);
  analogWrite(motorBackLeftPin, 0);
  delay(2000);
}

void leftReadSensor(){
  digitalWrite(lS2, LOW);
  digitalWrite(lS3, LOW);
  lredValue = pulseIn(lOUT, LOW);

  digitalWrite(lS2, HIGH);
  digitalWrite(lS3, HIGH);
  lgreenValue = pulseIn(lOUT, LOW) * 1.5;

  digitalWrite(lS2, LOW);
  digitalWrite(lS3, HIGH);
  lblueValue = pulseIn(lOUT, LOW);

  if (lgreenValue < lredValue && lgreenValue) {
    left_color = true;
  } else {
    left_color = false;
  }
}

void rightReadSensor(){
  digitalWrite(rS2, LOW);
  digitalWrite(rS3, LOW);
  rredValue = pulseIn(rOUT, LOW);

  digitalWrite(rS2, HIGH);
  digitalWrite(rS3, HIGH);
  rgreenValue = pulseIn(rOUT, LOW) * 1.2;

  digitalWrite(rS2, LOW);
  digitalWrite(rS3, HIGH);
  rblueValue = pulseIn(rOUT, LOW);

  if (rgreenValue < rredValue && rgreenValue) {
    right_color = true;
  } else {
    right_color = false;
  }
}

void setup() {
  pinMode(lS0, OUTPUT);
  pinMode(lS1, OUTPUT);
  pinMode(lS2, OUTPUT);
  pinMode(lS3, OUTPUT);
  pinMode(lOUT, INPUT);

  digitalWrite(lS0, HIGH);
  digitalWrite(lS1, LOW);

  pinMode(rS0, OUTPUT);
  pinMode(rS1, OUTPUT);
  pinMode(rS2, OUTPUT);
  pinMode(rS3, OUTPUT);
  pinMode(lOUT, INPUT);

  digitalWrite(rS0, HIGH);
  digitalWrite(rS1, LOW);

  pinMode(motorLeftPin, OUTPUT);
  pinMode(motorRightPin, OUTPUT);
  
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  if (left_color && right_color) {
    uturn();
  } else if (left_color) {
    forward();
    left();
  } else if (right_color) {
    forward();
    right();
  } else {
    analogWrite(motorBackRightPin, 0);
    analogWrite(motorBackLeftPin, 0);
  }

  calculatePID();
  delay(50);
}

long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}
