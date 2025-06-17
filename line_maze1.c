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

#define BUTTON_EXTRA 7 // D7 buat reset memori

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

// --- Variabel Path Baru ---
int pathlength = 0; // variable to record the total of path length
int readpath = 0;   // variable to call the path record (unused in this logic, but kept for declaration)
char path[25];      // array for path record, max 24 chars + null terminator
// --- Akhir Variabel Path Baru ---

bool readyToSavePath = false;
char pendingPath = '\0';
bool justDidUTurn = false;
bool justDidLeftTurn = false; // Flag untuk belok kiri
bool justDidRightTurn = false; // Flag untuk belok kanan
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

  resetMemory(); // Inisialisasi path saat startup
}

void loop() {
  if (digitalRead(BUTTON_EXTRA) == LOW) {
    resetMemory();
    // Tambahkan delay kecil untuk menghindari deteksi berulang jika tombol ditekan lama
    delay(200); 
  }
  readSensors();
  if (!isTurning) navigate();
  delay(10);
}

void resetMemory() {
  // Menggunakan memset untuk mengosongkan array path dan mengisi dengan null terminator
  memset(path, '\0', sizeof(path));
  pathlength = 0; // Reset panjang path
  readpath = 0;   // Reset readpath (jika digunakan untuk membaca path yang tersimpan)

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
    // Jika tidak ada sensor yang melihat garis, robot mungkin keluar jalur atau di persimpangan
    // Pertahankan error terakhir untuk antisipasi kehilangan garis
    error = lastError; 
  }

  integral += error;
  derivative = error - lastError;
  float correction = Kp * error + Ki * integral + Kd * derivative;
  
  // Perhitungan kecepatan motor diubah menjadi lebih jelas
  int leftSpeed = constrain(BASE_SPEED - correction, 0, 255); // Motor kecepatan max 255
  int rightSpeed = constrain(BASE_SPEED + correction, 0, 255); // Motor kecepatan max 255

  analogWrite(motorKananMaju, rightSpeed);
  analogWrite(motorKananMundur, 0); // Asumsi hanya maju
  analogWrite(motorKiriMaju, leftSpeed);
  analogWrite(motorKiriMundur, 0); // Asumsi hanya maju
  lastError = error;

  // Set flags untuk potensi penyimpanan path
  readyToSavePath = false;
  pendingPath = '\0';
  justDidUTurn = false;
  justDidLeftTurn = false;
  justDidRightTurn = false;


  // --- Logika Deteksi dan Penggerak Robot ---
  if (sensorStates == 0b00011000 || sensorStates == 0b00010000 || sensorStates == 0b00001000) {
    // Robot di tengah garis atau sedikit menyimpang (Pusat sensor 3 & 4 aktif)
    moveStraight();
    wasOnLine = true; // Konfirmasi robot di atas garis
  } else if (sensorStates == 0b11111111 || sensorStates == 0b01111110 || sensorStates == 0b01111111 || sensorStates == 0b11111110) {
    // Finish Line - semua atau hampir semua sensor aktif
    finishLine();
  } else if (sensorStates == 0b10011001 || sensorStates == 0b11011011 || sensorStates == 0b10010001 || sensorStates == 0b10001001 || sensorStates == 0b01011010 || sensorStates == 0b01011011 || sensorStates == 0b11011010 || sensorStates == 0b10010010 || sensorStates == 0b01010010 || sensorStates == 0b01011001) {
    // Perempatan - berbagai kombinasi sensor aktif di kedua sisi luar dan tengah
    intersection4Way(); 
    pendingPath = 'S'; // Asumsi default untuk perempatan adalah lurus (S), bisa diubah
    readyToSavePath = true;
  } else if (sensorStates == 0b10000001 || sensorStates == 0b01000010 || sensorStates == 0b11000011 || sensorStates == 0b01000011 || sensorStates == 0b11000010) {
    // Simpang 3 T - sensor luar aktif tanpa sensor tengah terlalu banyak
    intersection3WayT(); 
    pendingPath = 'L'; // Asumsi default untuk simpang T adalah belok kiri, bisa diubah
    readyToSavePath = true;
  } else if (sensorStates == 0b10011000 || sensorStates == 0b10010000 || sensorStates == 0b10001000 || sensorStates == 0b10000100 || sensorStates == 0b10001100 || sensorStates == 0b10110000 || sensorStates == 0b01010000 || sensorStates == 0b01011000 || sensorStates == 0b01001000 || sensorStates == 0b01011100 || sensorStates == 0b01001100 ) {
    // Simpang 3 Kiri - sensor di sisi kiri dan tengah-kanan aktif
    intersection3WayLeft(); 
    pendingPath = 'L'; // Belok kiri
    readyToSavePath = true;
  } else if (sensorStates == 0b00011001 || sensorStates == 0b00010001 || sensorStates == 0b00001001 || sensorStates == 0b00100001 || sensorStates == 0b00110001 || sensorStates == 0b00001101 || sensorStates == 0b00001010 || sensorStates == 0b00011010 || sensorStates == 0b00010010 || sensorStates == 0b00111010 || sensorStates == 0b00110010 ) {
    // Simpang 3 Kanan - sensor di sisi kanan dan tengah-kiri aktif
    intersection3WayRight(); 
    pendingPath = 'S'; // Asumsi lurus atau kanan
    readyToSavePath = true;
  } else if (((sensorStates & 0b11000000) > 0) && (sensorStates & 0b00011000) == 0) { // Sensor paling kiri (7,6) aktif, tengah mati
    // Belok Kiri ekstrim
    turnLeft();
    justDidLeftTurn = true;
  } else if (((sensorStates & 0b00000011) > 0) && (sensorStates & 0b00011000) == 0) { // Sensor paling kanan (1,0) aktif, tengah mati
    // Belok Kanan ekstrim
    turnRight();
    justDidRightTurn = true;
  } else if (sensorStates == 0b00000000 && wasOnLine && !justDidUTurn) {
    // Kehilangan garis setelah sebelumnya di atas garis, lakukan U-Turn
    uTurn();
    pendingPath = 'U'; 
    readyToSavePath = true;
    justDidUTurn = true; // Set flag untuk mencegah U-Turn berulang
  } else {
    // Default jika tidak ada kondisi spesifik terpenuhi, tetap jalankan PID
    // ini penting agar robot tidak berhenti jika sensor membaca sesuatu yang tidak terdefinisi
    moveStraight(); 
  }

  // --- Logika Penyimpanan Path Baru ---
  // Simpan karakter pendingPath hanya jika robot kembali ke garis lurus (sensor tengah aktif)
  // dan pathlength masih kurang dari batas array.
  if (readyToSavePath && (pathlength < (sizeof(path) - 1)) && 
      (sensorStates == 0b00011000 || sensorStates == 0b00010000 || sensorStates == 0b00001000) &&
      pendingPath != '\0') {
    path[pathlength] = pendingPath;
    pathlength++;
    path[pathlength] = '\0'; // Pastikan null-terminated
    Serial.print("Path ditambahkan: ");
    Serial.println(path);
    readyToSavePath = false; // Reset flag setelah menyimpan
    pendingPath = '\0'; // Reset karakter pending
    justDidUTurn = false; // Reset U-Turn flag setelah kembali ke garis
    justDidLeftTurn = false; // Reset flag belok kiri
    justDidRightTurn = false; // Reset flag belok kanan
  }
  // --- Akhir Logika Penyimpanan Path Baru ---

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
  display.println(path); // Menampilkan array char secara langsung
  display.display();
}

void moveStraight() {
  analogWrite(motorKananMaju, BASE_SPEED);
  analogWrite(motorKiriMaju, BASE_SPEED);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, 0);
  // Serial.println("Lurus"); // Dikomentari agar tidak terlalu banyak output serial
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
  analogWrite(motorKananMundur, BASE_SPEED / 1.6); // Putar di tempat atau hampir
  analogWrite(motorKiriMundur, 0);
  unsigned long timeout = millis() + 1200;
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 2); // Sedikit maju setelah belok
  analogWrite(motorKiriMaju, BASE_SPEED / 2);
  delay(100); // Pastikan robot sudah stabil di garis
  isTurning = false;
  integral = 0; // Reset integral term setelah belok
}

void turnLeft() {
  isTurning = true;
  Serial.println("Belok Kiri");
  currentDirection = "Belok Kiri";
  currentStatus = "Belok";
  analogWrite(motorKananMaju, BASE_SPEED / 1.6);
  analogWrite(motorKiriMaju, 0);
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, BASE_SPEED / 1.6); // Putar di tempat atau hampir
  unsigned long timeout = millis() + 1200;
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 2); // Sedikit maju setelah belok
  analogWrite(motorKiriMaju, BASE_SPEED / 2);
  delay(100); // Pastikan robot sudah stabil di garis
  isTurning = false;
  integral = 0; // Reset integral term setelah belok
}

void uTurn() {
  isTurning = true;
  Serial.println("U-Turn");
  currentDirection = "U-Turn";
  currentStatus = "Putar Balik";
  analogWrite(motorKananMaju, BASE_SPEED / 1.6); // Motor kanan maju
  analogWrite(motorKiriMundur, BASE_SPEED / 1.6); // Motor kiri mundur untuk putar di tempat
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMaju, 0);
  unsigned long timeout = millis() + 2000; // Timeout lebih lama untuk U-Turn
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 3); // Kecepatan lambat setelah U-Turn
  analogWrite(motorKiriMaju, BASE_SPEED / 3);
  delay(100);
  isTurning = false;
  integral = 0; // Reset integral term setelah U-Turn
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

  // Logika belok kiri untuk simpang (asumsi default belok kiri di simpang)
  // Perlu disesuaikan jika intersectionType berbeda handling-nya (e.g., simpang 3 kanan belok kanan)
  analogWrite(motorKananMaju, BASE_SPEED / 1.6); // motor kanan maju
  analogWrite(motorKiriMaju, 0);                 // motor kiri berhenti/mundur
  analogWrite(motorKananMundur, 0);
  analogWrite(motorKiriMundur, BASE_SPEED / 1.6); // motor kiri mundur
  
  unsigned long timeout = millis() + 1500; // Timeout yang lebih fleksibel untuk persimpangan
  // Tunggu hingga robot kembali ke garis lurus (sensor tengah aktif)
  while (!((sensorStates & 0b00010000) || (sensorStates & 0b00001000)) && millis() < timeout) {
    readSensors();
  }
  analogWrite(motorKananMaju, BASE_SPEED / 2); // Sedikit maju setelah belok
  analogWrite(motorKiriMaju, BASE_SPEED / 2);
  delay(100);
  isTurning = false;
  integral = 0; // Reset integral term setelah belok
}
