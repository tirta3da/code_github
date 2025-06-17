#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Display Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// OLED I2C Pins
#define OLED_SDA A4
#define OLED_SCL A5

// Button Pins
#define BUTTON_RIGHT 16  // A2
#define BUTTON_LEFT 17   // A3
#define BUTTON_OK 5      // D5
#define BUTTON_CANCEL 6  // D6
#define BUTTON_EXTRA 7  // D7 (used as reset)

// Multiplexer Pins
#define MUX_A 2
#define MUX_B 3
#define MUX_C 4
#define MUX_COM A0

// Motor Driver Pins
#define MOTOR_RIGHT_IN1 8
#define MOTOR_RIGHT_IN2 9
#define MOTOR_LEFT_IN1 10
#define MOTOR_LEFT_IN2 11

// Buzzer Pin
#define BUZZER 12

// Menu States
enum MenuState { MAIN_MENU, NAVIGASI, LINE_FOLLOWER, PID_KONTROL };
MenuState currentMenu = MAIN_MENU;
int selectedBox = 0;
const int totalBox = 3; // Including PID Control menu
bool inSubMenu = false;
int subMenuIndex = 0;
bool isLoading = true;

// Navigation Modes
enum NavigasiMode { FOLLOW_LINE, LINE_MASS };
NavigasiMode currentNavigasiMode = FOLLOW_LINE; // Default to FOLLOW_LINE

// PID Parameters
struct PID {
  double Kp = 10.0;
  double Ki = 0.5;
  double Kd = 5.0;
  double error = 0;
  double lastError = 0;
  double integral = 0;
  double derivative = 0;
} pid;

// Line Follower Variables
const int numSensors = 8;
int sensorValues[8];
int pwmLeft = 150;
int pwmRight = 150;
int baseSpeed = 120;
bool running = false;
byte activeParam = 0;// Save PID parameters to EEPROM
void savePIDToEEPROM() {
  EEPROM.put(0, pid.Kp);
  EEPROM.put(8, pid.Ki);
  EEPROM.put(16, pid.Kd);
}

// Read PID parameters from EEPROM
void readPIDFromEEPROM() {
  pid.Kp = EEPROM.get(0, pid.Kp);
  pid.Ki = EEPROM.get(8, pid.Ki);
  pid.Kd = EEPROM.get(16, pid.Kd);
}

// Reset PID parameters to default values
void resetPID() {
  pid.Kp = 10.0;
  pid.Ki = 0.5;
  pid.Kd = 5.0;
  savePIDToEEPROM(); // Save default PID values to EEPROM
}

// Save route data to EEPROM
void saveRouteToEEPROM(int* route, int size) {
  for (int i = 0; i < size; i++) {
    EEPROM.put(i * sizeof(int), route[i]);
  }
}

// Read route data from EEPROM
void readRouteFromEEPROM(int* route, int size) {
  for (int i = 0; i < size; i++) {
    route[i] = EEPROM.get(i * sizeof(int), route[i]);
  }
}

// Reset route data in EEPROM to zero
void resetRouteInEEPROM(int size) {
  for (int i = 0; i < size; i++) {
    EEPROM.put(i * sizeof(int), 0); // Reset all route data to 0
  }
}void setup() {
  // Initialize button pins with internal pull-up resistors
  pinMode(BUTTON_RIGHT, INPUT_PULLUP);
  pinMode(BUTTON_LEFT, INPUT_PULLUP);
  pinMode(BUTTON_OK, INPUT_PULLUP);
  pinMode(BUTTON_CANCEL, INPUT_PULLUP);
  pinMode(BUTTON_EXTRA, INPUT_PULLUP); // Reset button on D7

  // Initialize multiplexer pins
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  pinMode(MUX_C, OUTPUT);

  // Initialize motor driver pins
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);

  // Initialize buzzer pin
  pinMode(BUZZER, OUTPUT);

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for (;;); // Halt if display initialization fails
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.dim(false);

  // Play startup tone and show loading screen
  playButtonTone();
  tampilLoading();

  // Load PID values from EEPROM
  readPIDFromEEPROM();
}

void loop() {
  static bool lastRight = HIGH, lastLeft = HIGH, lastOK = HIGH, lastCancel = HIGH, lastExtra = HIGH;
  static unsigned long lastButtonTime = 0;
  const unsigned long debounceDelay = 200;

  // Read button states
  bool currentRight = digitalRead(BUTTON_RIGHT);
  bool currentLeft = digitalRead(BUTTON_LEFT);
  bool currentOK = digitalRead(BUTTON_OK);
  bool currentCancel = digitalRead(BUTTON_CANCEL);
  bool currentExtra = digitalRead(BUTTON_EXTRA);
  unsigned long currentTime = millis();

  display.clearDisplay();

  // Handle RIGHT button (navigate forward or increase PID)
  if (currentRight == LOW && lastRight == HIGH && currentTime - lastButtonTime > debounceDelay) {
    playButtonTone();
    if (currentMenu == MAIN_MENU) {
      selectedBox = (selectedBox + 1) % totalBox;
    } else if (currentMenu == NAVIGASI && inSubMenu) {
      subMenuIndex = (subMenuIndex + 1) % 6;
    } else if (currentMenu == PID_KONTROL) {
      double increment = (activeParam == 1) ? 0.01 : 0.1;
      if (activeParam == 0) pid.Kp += increment;
      else if (activeParam == 1) pid.Ki += increment;
      else pid.Kd += increment;
      savePIDToEEPROM();
    }
    lastButtonTime = currentTime;
  }

  // Handle LEFT button (navigate backward or decrease PID)
  if (currentLeft == LOW && lastLeft == HIGH && currentTime - lastButtonTime > debounceDelay) {
    playButtonTone();
    if (currentMenu == MAIN_MENU) {
      selectedBox = (selectedBox - 1 + totalBox) % totalBox;
    } else if (currentMenu == NAVIGASI && inSubMenu) {
      subMenuIndex = (subMenuIndex - 1 + 6) % 6;
    } else if (currentMenu == PID_KONTROL) {
      double decrement = (activeParam == 1) ? 0.01 : 0.1;
      if (activeParam == 0) pid.Kp = max(0.0, pid.Kp - decrement);
      else if (activeParam == 1) pid.Ki = max(0.0, pid.Ki - decrement);
      else pid.Kd = max(0.0, pid.Kd - decrement);
      savePIDToEEPROM();
    }
    lastButtonTime = currentTime;
  }

  // Handle OK button (select menu or cycle PID parameter)
  if (currentOK == LOW && lastOK == HIGH && currentTime - lastButtonTime > debounceDelay) {
    playButtonTone();
    if (currentMenu == MAIN_MENU) {
      if (selectedBox == 0) currentMenu = NAVIGASI;
      else if (selectedBox == 1) currentMenu = LINE_FOLLOWER;
      else if (selectedBox == 2) currentMenu = PID_KONTROL;
    } else if (currentMenu == NAVIGASI && subMenuIndex == 2) {
      inSubMenu = true;
    } else if (currentMenu == PID_KONTROL) {
      activeParam = (activeParam + 1) % 3;
    }
    lastButtonTime = currentTime;
  }

  // Handle CANCEL button (exit submenu or return to main menu)
  if (currentCancel == LOW && lastCancel == HIGH && currentTime - lastButtonTime > debounceDelay) {
    playButtonTone();
    if (currentMenu == NAVIGASI && inSubMenu) {
      inSubMenu = false;
    } else if (currentMenu != MAIN_MENU) {
      currentMenu = MAIN_MENU;
    }
    lastButtonTime = currentTime;
  }

  // Handle EXTRA button (reset PID and route)
  if (currentExtra == LOW && lastExtra == HIGH && currentTime - lastButtonTime > debounceDelay) {
    playButtonTone();
    resetPID();
    resetRouteInEEPROM(100); // Reset stored route in EEPROM
    lastButtonTime = currentTime;
  }

  // Update display based on current menu
  switch (currentMenu) {
    case MAIN_MENU:
      tampilMainMenu();
      break;
    case NAVIGASI:
      if (inSubMenu) tampilSubMenu();
      else tampilMenuNavigasi();
      break;
    case LINE_FOLLOWER:
      if (running) {
        updateLineFollower();
        updateMotors();
      } else {
        display.setCursor(10, SCREEN_HEIGHT - 20);
        display.print(F("Press EXTRA to start"));
      }
      tampilLineFollower();
      break;
    case PID_KONTROL:
      tampilPidKontrol();
      break;
  }

  // Update button states
  lastRight = currentRight;
  lastLeft = currentLeft;
  lastOK = currentOK;
  lastCancel = currentCancel;
  lastExtra = currentExtra;

  display.display();
}void playButtonTone() {
  tone(BUZZER, 1000, 50);
}

void tampilLoading() {
  const int kotakJumlah = 6, totalLoop = 3;
  for (int loop = 0; loop < totalLoop; loop++) {
    for (int i = 0; i < kotakJumlah; i++) {
      display.clearDisplay();
      for (int j = 0; j < kotakJumlah; j++) {
        int x = 10 + j * 18;
        if (i == j) display.fillRect(x, 30, 15, 15, SSD1306_WHITE);
        else display.drawRect(x, 30, 15, 15, SSD1306_WHITE);
      }
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(35, 10);
      display.print("Tim Robotika");
      display.display();
      delay(200);
    }
  }
  isLoading = false;
  tampilkanNamaKelompok();
  delay(2000);
  currentMenu = MAIN_MENU;
  tampilMainMenu();
  display.display();
}

void tampilkanNamaKelompok() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 0);
  display.println("KELOMPOK 3");
  display.setCursor(5, 15); display.println("FARHAN");
  display.setCursor(5, 25); display.println("SYAFIQ");
  display.setCursor(5, 35); display.println("TIRTA");
  display.setCursor(5, 45); display.println("ALFI");
  display.setCursor(5, 55); display.println("WULAN");
  display.display();
}

void tampilMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(40, 0);
  display.print("MAIN MENU");
  display.drawLine(0, 10, SCREEN_WIDTH - 1, 10, SSD1306_WHITE);

  // Menu Line Mass
  if (selectedBox == 0) {
    display.fillRect(20, 15, 88, 15, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(20, 15, 88, 15, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(40, 18);
  display.print("LINE MASS");

  // Mode Line Follower
  if (selectedBox == 1) {
    display.fillRect(20, 32, 88, 15, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(20, 32, 88, 15, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(25, 35);
  display.print("LINE FOLLOWER");

  // Mode PID Control
  if (selectedBox == 2) {
    display.fillRect(20, 49, 88, 15, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
  } else {
    display.drawRect(20, 49, 88, 15, SSD1306_WHITE);
    display.setTextColor(SSD1306_WHITE);
  }
  display.setCursor(30, 52);
  display.print("PID KONTROL");
}

void tampilKonfirmasiMode(String mode) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 20);
  display.print("Apakah Anda yakin?");
  display.setCursor(20, 40);
  display.print("Mode: ");
  display.print(mode);
  display.setCursor(20, 60);
  display.print("OK untuk Mulai");
  display.display();
}

void tampilSubMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 0);
  display.print("OTHER SETTINGS");
  display.display();
}

void tampilPidKontrol() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(35, 0);
  display.print("PID KONTROL");

  const char* params[] = {"Kp", "Ki", "Kd"};
  double* values[] = {&pid.Kp, &pid.Ki, &pid.Kd};
  const byte decimals[] = {2, 3, 2};

  for (byte i = 0; i < 3; i++) {
    int yOffset = 15 + (i * 12);
    display.setCursor(10, yOffset);
    display.print(i == activeParam ? "> " : "  ");
    display.print(params[i]);
    display.print(": ");
    display.print(*values[i], decimals[i]);
  }
}

void tampilMenuNavigasi() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Show progress
  display.setCursor(0, 0);
  display.print("Progress: ");
  display.print("100%");

  // Display navigation mode options
  display.setTextColor(subMenuIndex == 0 ? SSD1306_BLACK : SSD1306_WHITE);
  display.fillRect(0, 10, 60, 25, subMenuIndex == 0 ? SSD1306_WHITE : SSD1306_BLACK);
  display.setCursor(2, 12);
  display.print("START CP");

  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 18);
  display.print("00");

  // Mode selection
  display.setTextSize(1);
  display.setTextColor(subMenuIndex == 1 ? SSD1306_BLACK : SSD1306_WHITE);
  display.fillRect(65, 10, 60, 12, subMenuIndex == 1 ? SSD1306_WHITE : SSD1306_BLACK);
  display.setCursor(67, 12);
  display.print("Mode Garis");

  display.setTextColor(subMenuIndex == 2 ? SSD1306_BLACK : SSD1306_WHITE);
  display.fillRect(65, 23, 60, 12, subMenuIndex == 2 ? SSD1306_WHITE : SSD1306_BLACK);
  display.setCursor(67, 25);
  display.print("Cari Rute");
}void updateLineFollower() {
  // Read sensor values from multiplexer
  for (int i = 0; i < numSensors; i++) {
    digitalWrite(MUX_A, (i & 0x01) ? HIGH : LOW);
    digitalWrite(MUX_B, (i & 0x02) ? HIGH : LOW);
    digitalWrite(MUX_C, (i & 0x04) ? HIGH : LOW);
    delayMicroseconds(10);
    sensorValues[i] = analogRead(MUX_COM) > 500 ? 1 : 0;
  }

  // Calculate line position
  int sum = 0, count = 0;
  for (int i = 0; i < numSensors; i++) {
    if (sensorValues[i] == 1) {
      sum += i * 1000;
      count++;
    }
  }

  int linePosition = count > 0 ? sum / count : 3500;
  pid.error = linePosition - 3500;

  // Update PID calculations
  pid.integral += pid.error;
  pid.derivative = pid.error - pid.lastError;
  pid.lastError = pid.error;
  double output = (pid.Kp * pid.error) + (pid.Ki * pid.integral) + (pid.Kd * pid.derivative);

  // Adjust motor speeds
  pwmLeft = constrain(baseSpeed + (int)output, 0, 255);
  pwmRight = constrain(baseSpeed - (int)output, 0, 255);
}

void updateMotors() {
  // Control right motor
  if (pwmRight > 0) {
    analogWrite(MOTOR_RIGHT_IN1, pwmRight);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
  }

  // Control left motor
  if (pwmLeft > 0) {
    analogWrite(MOTOR_LEFT_IN1, pwmLeft);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  } else {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
  }
}

void tampilLineFollower() {
  display.clearDisplay();

  // Display sensor bars
  int barWidth = 15, maxBarHeight = SCREEN_HEIGHT / 4, barSpacing = 1, startX = 0, startY = maxBarHeight;
  for (int i = 0; i < numSensors; i++) {
    int barHeight = sensorValues[i] ? maxBarHeight : 3;
    display.fillRect(startX + i * (barWidth + barSpacing), startY - barHeight, barWidth, barHeight, SSD1306_WHITE);
  }
  display.drawLine(0, 18, SCREEN_WIDTH - 1, 18, SSD1306_WHITE);

  // Display motor speeds
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  String motorText = "L " + String(pwmLeft) + " R " + String(pwmRight);
  int textWidth = motorText.length() * 6;
  int motorTextX = (SCREEN_WIDTH - textWidth) / 2, motorTextY = 22;
  display.drawRect(motorTextX - 4, motorTextY - 4, textWidth + 8, 16 + 8, SSD1306_WHITE);
  display.setCursor(motorTextX, motorTextY);
  display.setTextColor(SSD1306_INVERSE);
  display.print(motorText);

  // Display PID error
  String errorText = "Err:" + String(pid.error >= 0 ? "+" : "") + String(pid.error);
  textWidth = errorText.length() * 6;
  int errorTextX = (SCREEN_WIDTH - textWidth) / 2;
  display.setCursor(errorTextX, 44);
  display.setTextColor(SSD1306_WHITE);
  display.print(errorText);

  // Display base speed
  String speedText = "Spd:" + String(baseSpeed);
  textWidth = speedText.length() * 6;
  int speedTextX = (SCREEN_WIDTH - textWidth) / 2;
  display.setCursor(speedTextX, 54);
  display.print(speedText);

  // Draw borders
  display.drawLine(0, 18, 0, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.drawLine(SCREEN_WIDTH - 1, 18, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.drawLine(0, SCREEN_HEIGHT - 1, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1, SSD1306_WHITE);
  display.drawLine(0, 42, SCREEN_WIDTH - 1, 42, SSD1306_WHITE);
  display.drawLine(0, 52, SCREEN_WIDTH - 1, 52, SSD1306_WHITE);
}

void lineMass() {
  // Follow left path to finish
  followLeftPath();

  // Reverse and follow right path
  followRightPath();

  // Save route to EEPROM
  saveRouteToEEPROM(sensorValues, numSensors);

  // Display confirmation
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Rute berhasil disimpan!");
  display.display();
  delay(2000);
}

void followLeftPath() {
  while (!reachedFinish()) {
    moveForward();
    if (detectLeftLine()) {
      turnLeft();
    }
  }
}

void followRightPath() {
  while (!reachedStart()) {
    moveForward();
    if (detectRightLine()) {
      turnRight();
    }
  }
}

void recallRoute() {
  int route[100];
  readRouteFromEEPROM(route, 100);

  // Follow stored route
  for (int i = 0; i < 100; i++) {
    if (route[i] == 1) {
      moveForward();
    } else if (route[i] == 0) {
      stopMotor();
    }
  }
}

void navigasiModeHandler() {
  if (subMenuIndex == 1) {
    currentNavigasiMode = FOLLOW_LINE;
    display.setCursor(0, SCREEN_HEIGHT - 20);
    display.print("Mode: FOLLOW LINE");
  } else if (subMenuIndex == 2) {
    currentNavigasiMode = LINE_MASS;
    display.setCursor(0, SCREEN_HEIGHT - 20);
    display.print("Mode: LINE MASS");
  }
}

void cariRuteTerdekat() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Mencari Rute...");
  String ruteText = "Rute ditemukan!";
  display.setCursor(0, 20);
  display.print(ruteText);
  display.display();
}

// Placeholder functions (to be implemented)
bool reachedFinish() { return false; }  // Implement logic to detect finish
bool reachedStart() { return false; }   // Implement logic to detect start
bool detectLeftLine() { return false; } // Implement left line detection
bool detectRightLine() { return false; } // Implement right line detection
void moveForward() {}                   // Implement forward movement
void turnLeft() {}                      // Implement left turn
void turnRight() {}                     // Implement right turn
void stopMotor() {}                     // Implement motor stop



//itu mana yang salah, kenapa ketika line mass yang dipilih tidak bisa memilih lagi 
