#include <Servo.h>

const int buttonPin = 24;
const int potentiometerPin = A0;//raspberry pi pico A0=26
const int manualLED = 25;
const int centerLED = 4;
const int autoLED = 5;

 //如果你是飛控舵機預設使用60~120度 servoAngle則輸入30
 //如果你是SG92R舵機預設使用可以0~180度則 servoAngle則輸入0

 
const int servoAngle=30;
//const int servoAngle=0;
const int servoPin = 18;
const int angleMin = servoAngle;
const int angleMax = 180 - servoAngle;

Servo myservo;
int angle = 0;
int mode = 0;
int buttonState = 0;
int lastButtonState = 0;
int servo_dir = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(manualLED, OUTPUT);
  pinMode(centerLED, OUTPUT);
  pinMode(autoLED, OUTPUT);
  //pinMode(potentiometerPin, INPUT);
  myservo.attach(servoPin);

  setMode(0);
}

void loop() {
  int reading = digitalRead(buttonPin);
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        mode = (mode + 1) % 3;
        setMode(mode);
      }
    }
  }

  switch (mode) {
    case 0:
      manualMode();//手動模式
      break;
    case 1:
      centerMode();//舵機置中
      break;
    case 2:
      autoMode();//自動掃蕩
      break;
  }

  lastButtonState = reading;
}

void setMode(int mode) {
  digitalWrite(manualLED, mode == 0 ? HIGH : LOW);
  digitalWrite(centerLED, mode == 1 ? HIGH : LOW);
  digitalWrite(autoLED, mode == 2 ? HIGH : LOW);
  Serial.println(mode);
}

void manualMode() {
  int potValue = analogRead(potentiometerPin);
  int angle = map(potValue, 0, 1024, angleMin, angleMax);
  Serial.println(angle);
  myservo.write(angle);
}

void centerMode() {
  myservo.write(90);
}

void autoMode() {

  if (servo_dir == 0) {
    if (angle < angleMax) {
      angle=angle+2;

    } else {
      servo_dir = 1;
    }
  } else {
    if (angle > angleMin) {
      angle=angle-2;
    } else {
      servo_dir = 0;
    }
  }
  Serial.println(angle);

  myservo.write(angle);
  delay(25);
}
