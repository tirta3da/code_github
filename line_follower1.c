#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED Configuration
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Multiplexer Pins
const int selectPins[3] = {2, 3, 4};
const int analogPin = A0;

// Sensor Thresholds & Variables
int thresholds[8] = {802, 752, 700, 677, 650, 742, 707, 782};
int sensorValues[8];
bool sensorActive[8];

// Motor Pins
const int motorKananMaju = 6;
const int motorKananMundur = 9;
const int motorKiriMaju = 11;
const int motorKiriMundur = 10;

// Motion Constants
const int BASE_SPEED_kiri = 140;
const int BASE_SPEED_kanan = 140;

// PID Constants
float Kp = 16;
float Ki = 0;
float Kd = 55;

float error;
float lastError;
float integral;

// Weights for the sensor readings
int weights[8] = {-7, -4.5, -1.5, -0.5, 0.5, 1.5, 4.5, 7};

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
}

void loop() {
  readSensors();  // Membaca sensor
  displayReadings();  // Menampilkan pembacaan sensor ke OLED

  pidControlLogic(); // Menghitung kontrol PID untuk menggerakkan motor

  delay(50);
}

// ========== SENSOR READING ==========
// Fungsi untuk membaca sensor line
void readSensors() {
  for (int i = 0; i < 8; i++) {
    setMultiplexerChannel(i);
    delayMicroseconds(50);
    int value = (analogRead(analogPin) + analogRead(analogPin)) / 2;
    sensorValues[i] = value;
    sensorActive[i] = value < thresholds[i];  // Sensor aktif jika nilai sensor lebih kecil dari threshold
  }
}

// ========== DISPLAY ==========
// Fungsi untuk menampilkan status sensor dalam bentuk biner ke OLED
void displayReadings() {
  display.clearDisplay();

  // Baris 1: Status sensor aktif (1/0) dalam format biner
  display.setCursor(0, 0);
  display.print("Sensor: ");
  for (int i = 0; i < 8; i++) {
    display.print(sensorActive[i] ? "1" : "0");
  }

  display.display();
}

// ========== PID CONTROL ==========
// Fungsi untuk menghitung PID dan mengontrol motor
void pidControlLogic() {
  int weightedSum = 0;
  int activeCount = 0;

  // Menghitung error berdasarkan sensor yang aktif dan bobot sensor
  for (int i = 0; i < 8; i++) {
    if (sensorActive[i]) {
      weightedSum += weights[i]; // Menambahkan bobot sensor
      activeCount++;
    }
  }

  // Jika ada sensor yang aktif, hitung error
  if (activeCount > 0) {
    error = weightedSum / (float)activeCount;
  } else {
    error = lastError;
  }

  integral += error;
  float derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;

  // Menghitung kecepatan motor kiri dan kanan berdasarkan koreksi PID
  int leftSpeed = BASE_SPEED_kiri - correction;
  int rightSpeed = BASE_SPEED_kanan + correction;

  leftSpeed = constrain(leftSpeed, 0,  BASE_SPEED_kiri);
  rightSpeed = constrain(rightSpeed, 0,  BASE_SPEED_kanan);

  // Mengatur motor berdasarkan hasil PID
  analogWrite(motorKananMaju, rightSpeed);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMaju, leftSpeed);
  analogWrite(motorKiriMundur, 0);

  // Debug serial untuk memantau nilai PID
  Serial.print("ERR: "); Serial.print(error);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.print(rightSpeed);
  Serial.print(" | KP: "); Serial.print(Kp);
  Serial.print(" KI: "); Serial.print(Ki);
  Serial.print(" KD: "); Serial.println(Kd);
}

// ========== MULTIPLEXER ==========
// Fungsi untuk memilih saluran sensor menggunakan multiplexer
void setMultiplexerChannel(int channel) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(selectPins[i], bitRead(channel, i));
  }
}
