#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

const int selectPins[3] = {2, 3, 4};
const int analogPin = A0;

int sensorStates = 0;
int thresholds[8] = {802, 752, 580, 677, 615, 742, 707, 782};

const int motorKananMaju = 9;
const int motorKananMundur = 6;
const int motorKiriMaju = 10;
const int motorKiriMundur = 11;

#define BUTTON_EXTRA 7

const int BASE_SPEED = 60;
float Kp = 10;
float Ki = 0.0001;
float Kd = 10 * Kp;

float error;
float lastError = 0;
float integral = 0;
float derivative;

int weights[8] = {-7, -5, -2.5, -1, 1, 2.5, 5, 7};

bool isTurning = false;
String currentDirection = "Standby";
String currentStatus = "Jalan";

int pathlength = 0;
int readpath = 0;
char path[25];

bool readyToSavePath = false;
char pendingPath = '\0';
bool justDidUTurn = false;
bool justDidLeftTurn = false;
bool justDidRightTurn = false;
bool wasOnLine = false;

void performIntersectionTurn(const char* intersectionType);
void moveStraight();
void turnRight();
void turnLeft();
void uTurn();
void finishLine();
void intersection3WayRight();
void intersection3WayLeft();
void intersection3WayT();
void intersection4Way();
void updateOLEDDisplay();
void resetMemory();

String simplifyPath(String path) {
  path.replace("SUL", "R");
  path.replace("LUL", "s");
  return path;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("OLED init failed"));
    while (true);
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  for (int i = 0; i < 3; i++) pinMode(selectPins[i], OUTPUT);
  pinMode(motorKananMaju, OUTPUT);
  pinMode(motorKananMundur, OUTPUT);
  pinMode(motorKiriMaju, OUTPUT);
  pinMode(motorKiriMundur, OUTPUT);

  pinMode(BUTTON_EXTRA, INPUT_PULLUP);

  resetMemory();
}

void loop() {
  if (digitalRead(BUTTON_EXTRA) == LOW) {
    resetMemory();
    delay(200);
  }
  readSensors();
  if (!isTurning) navigate();
  delay(10);
}

void resetMemory() {
  memset(path, '\0', sizeof(path));
  pathlength = 0;
  readpath = 0;

  currentDirection = "Standby";
  currentStatus = "Reset";
  readyToSavePath = false;
  pendingPath = '\0';
  justDidUTurn = false;
  justDidLeftTurn = false;
  justDidRightTurn = false;
  updateOLEDDisplay();
  Serial.println("Path di-reset!");
}

void readSensors() {
  sensorStates = 0;
  for (int i = 0; i < 8; i++) {
    setMultiplexerChannel(i);
    delayMicroseconds(50);
    int value = (analogRead(analogPin) + analogRead(analogPin)) / 2;
    if (value < thresholds[i]) {
      sensorStates |= (1 << i);
    }
  }
}

void setMultiplexerChannel(int channel) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(selectPins[i], bitRead(channel, i));
  }
}

void navigate() {
  int weightedSum = 0;
  int activeCount = 0;
  for (int i = 0; i < 8; i++) {
    if (sensorStates & (1 << i)) {
      weightedSum += weights[i];
      activeCount++;
    }
  }

  if (activeCount > 0) {
    error = weightedSum / (float)activeCount;
    if (error * lastError < 0) integral = 0;
  } else {
    error = lastError;
  }

  integral += error;
  derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;

  int leftSpeed = constrain(BASE_SPEED - correction, 0, 255);
  int rightSpeed = constrain(BASE_SPEED + correction, 0, 255);

  analogWrite(motorKananMaju, rightSpeed);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMaju, leftSpeed);
  analogWrite(motorKiriMundur, 0);
  lastError = error;

  readyToSavePath = false;
  pendingPath = '\0';
  justDidUTurn = false;
  justDidLeftTurn = false;
  justDidRightTurn = false;

  if (sensorStates == 0b00011000 || sensorStates == 0b00010000 || sensorStates == 0b00001000) {
    moveStraight();
    wasOnLine = true;
  } else if (sensorStates == 0b11111111 || sensorStates == 0b01111110 || sensorStates == 0b01111111 || sensorStates == 0b11111110) {
    finishLine();
  } else if (sensorStates == 0b10011001 || sensorStates == 0b11011011 || sensorStates == 0b10010001 || sensorStates == 0b10001001 || sensorStates == 0b01011010 || sensorStates == 0b01011011 || sensorStates == 0b11011010 || sensorStates == 0b10010010 || sensorStates == 0b01010010 || sensorStates == 0b01011001) {
    intersection4Way();
    pendingPath = 'S';
    readyToSavePath = true;
  } else if (sensorStates == 0b10000001 || sensorStates == 0b01000010 || sensorStates == 0b11000011 || sensorStates == 0b01000011 || sensorStates == 0b11000010) {
    intersection3WayT();
    pendingPath = 'L';
    readyToSavePath = true;
  } else if (sensorStates == 0b10011000 || sensorStates == 0b10010000 || sensorStates == 0b10001000 || sensorStates == 0b10000100 || sensorStates == 0b10001100 || sensorStates == 0b10110000 || sensorStates == 0b01010000 || sensorStates == 0b01011000 || sensorStates == 0b01001000 || sensorStates == 0b01011100 || sensorStates == 0b01001100 ) {
    intersection3WayLeft();
    pendingPath = 'L';
    readyToSavePath = true;
  } else if (sensorStates == 0b00011001 || sensorStates == 0b00010001 || sensorStates == 0b00001001 || sensorStates == 0b00100001 || sensorStates == 0b00110001 || sensorStates == 0b00001101 || sensorStates == 0b00001010 || sensorStates == 0b00011010 || sensorStates == 0b00010010 || sensorStates == 0b00111010 || sensorStates == 0b00110010 ) {
    intersection3WayRight();
    pendingPath = 'S';
    readyToSavePath = true;
  } else if (((sensorStates & 0b11000000) > 0) && (sensorStates & 0b00011000) == 0) {
    turnLeft();
    justDidLeftTurn = true;
  } else if (((sensorStates & 0b00000011) > 0) && (sensorStates & 0b00011000) == 0) {
    turnRight();
    justDidRightTurn = true;
  } else if (sensorStates == 0b00000000 && wasOnLine && !justDidUTurn) {
    uTurn();
    pendingPath = 'U';
    readyToSavePath = true;
    justDidUTurn = true;
  } else {
    moveStraight();
  }

  if (readyToSavePath && (pathlength < (sizeof(path) - 1)) && 
      (sensorStates == 0b00011000 || sensorStates == 0b00010000 || sensorStates == 0b00001000) &&
      pendingPath != '\0') {
    path[pathlength] = pendingPath;
    pathlength++;
    path[pathlength] = '\0';
    Serial.print("Path ditambahkan: ");
    Serial.println(path);
    readyToSavePath = false;
    pendingPath = '\0';
    justDidUTurn = false;
    justDidLeftTurn = false;
    justDidRightTurn = false;
  }

  updateOLEDDisplay();
}

void updateOLEDDisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Sensor: ");
  for (int i = 7; i >= 0; i--) {
    display.print(bitRead(sensorStates, i));
  }
  display.setCursor(0, 10);
  display.print("Arah: ");
  display.println(currentDirection);
  display.setCursor(0, 20);
  display.print("Status: ");
  display.println(currentStatus);
  display.setCursor(0, 30);
  display.print("Path: ");
  display.println(path);
  display.display();
}

void moveStraight() {
  analogWrite(motorKananMaju, BASE_SPEED);
  analogWrite(motorKiriMaju, BASE_SPEED);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, 0);
  currentDirection = "Lurus";
  currentStatus = "Jalan";
}

void turnRight() {
  isTurning = true;
  Serial.println("Belok Kanan");
  currentDirection = "Belok Kanan";
  currentStatus = "Belok";
  analogWrite(motorKananMaju, 0);
  analogWrite(motorKiriMaju, BASE_SPEED / 1.6);
  analogWrite(motorKananMundur, BASE_SPEED / 1.6);
  analogWrite(motorKiriMundur, 0);
  unsigned long timeout = millis() + 1200;
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 2);
  analogWrite(motorKiriMaju, BASE_SPEED / 2);
  delay(100);
  isTurning = false;
  integral = 0;
}

void turnLeft() {
  isTurning = true;
  Serial.println("Belok Kiri");
  currentDirection = "Belok Kiri";
  currentStatus = "Belok";
  analogWrite(motorKananMaju, BASE_SPEED / 1.6);
  analogWrite(motorKiriMaju, 0);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, BASE_SPEED / 1.6);
  unsigned long timeout = millis() + 1200;
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 2);
  analogWrite(motorKiriMaju, BASE_SPEED / 2);
  delay(100);
  isTurning = false;
  integral = 0;
}

void uTurn() {
  isTurning = true;
  Serial.println("U-Turn");
  currentDirection = "U-Turn";
  currentStatus = "Putar Balik";
  analogWrite(motorKananMaju, BASE_SPEED / 1.6);
  analogWrite(motorKiriMundur, BASE_SPEED / 1.6);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMaju, 0);
  unsigned long timeout = millis() + 2000;
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 3);
  analogWrite(motorKiriMaju, BASE_SPEED / 3);
  delay(100);
  isTurning = false;
  integral = 0;
}

void finishLine() {
  Serial.println("Finish Line");
  analogWrite(motorKananMaju, 0);
  analogWrite(motorKiriMaju, 0);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, 0);
  currentStatus = "FINISH";
}

void intersection3WayRight() { performIntersectionTurn("Simpang 3R"); }
void intersection3WayLeft()  { performIntersectionTurn("Simpang 3L"); }
void intersection3WayT()     { performIntersectionTurn("Simpang 3T"); }
void intersection4Way()      { performIntersectionTurn("Perempatan"); }

void performIntersectionTurn(const char* intersectionType) {
  isTurning = true;
  Serial.print("Menuju ");
  Serial.println(intersectionType);
  currentDirection = intersectionType;
  currentStatus = "Belok";

  analogWrite(motorKananMaju, BASE_SPEED / 1.6);
  analogWrite(motorKiriMaju, 0);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, BASE_SPEED / 1.6);
  
  unsigned long timeout = millis() + 1500;
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 2);
  analogWrite(motorKiriMaju, BASE_SPEED / 2);
  delay(100);
  isTurning = false;
  integral = 0;
}
