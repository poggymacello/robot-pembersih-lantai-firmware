// =====================================================================
// USANG (OBSOLETE) - JANGAN DIPAKAI DI PCB UTAMA
// Sketch ini dari uji breadboard 15 Okt (lihat Hasil_Uji_PID_2026-10-15.md) dan hanya
// disimpan sebagai arsip. Pin 36/37 bentrok dengan PSRAM octal (N16R8) dan pin 38 dipakai
// LED R di PCB; encoder kanan 10/11 juga dipakai suction/ECHO US1 di PCB.
// Gunakan Uji_PID_Kecepatan_Roda_v2 (pin sesuai PCB rev 004).
// =====================================================================

// --- Definisi Pin Motor (TB6612FNG) ---
// Motor KIRI (Pin Baru)
const int pinPWMA = 38; 
const int pinAIN2 = 37; 
const int pinAIN1 = 36; 

// Motor KANAN (Tetap)
const int pinPWMB = 9;  
const int pinBIN1 = 18; 
const int pinBIN2 = 8;  

// --- Definisi Pin Encoder ---
const int pinKiriC1 = 4;
const int pinKiriC2 = 5;

// Motor Kanan (Menggunakan pin yang stabil: 10 & 11)
const int pinKananC1 = 10;
const int pinKananC2 = 11;

// --- Variabel Pembacaan Tick ---
volatile long tickKiri = 0;
volatile long tickKanan = 0;
long tickKiriLama = 0;
long tickKananLama = 0;

// --- Pengaturan Waktu (Sampling) ---
unsigned long waktuLama = 0;
const int interval = 50; 

// --- Parameter PID Final ---
float Kp = 3.0;  
float Ki = 1.5;  
float Kd = 2.0;  

// Berdasarkan log Anda, kecepatan stabil di sekitar angka 30
int targetSpeed = 30; 

// Variabel kalkulasi PID
float errorKiri = 0, lastErrorKiri = 0, integralKiri = 0;
float errorKanan = 0, lastErrorKanan = 0, integralKanan = 0;

// Fungsi Interrupt
void IRAM_ATTR bacaEncoderKiri() {
  if (digitalRead(pinKiriC2) == HIGH) tickKiri++;
  else tickKiri--;
}

void IRAM_ATTR bacaEncoderKanan() {
  if (digitalRead(pinKananC2) == HIGH) tickKanan++;
  else tickKanan--;
}

void setup() {
  Serial.begin(115200);

  // Setup Pin Encoder
  pinMode(pinKiriC1, INPUT_PULLUP);
  pinMode(pinKiriC2, INPUT_PULLUP);
  pinMode(pinKananC1, INPUT_PULLUP);
  pinMode(pinKananC2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(pinKiriC1), bacaEncoderKiri, RISING);
  attachInterrupt(digitalPinToInterrupt(pinKananC1), bacaEncoderKanan, RISING);

  // Setup Pin Motor
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinAIN2, OUTPUT);
  
  pinMode(pinPWMB, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);

  // Arah Maju
  digitalWrite(pinAIN1, HIGH);
  digitalWrite(pinAIN2, LOW);
  digitalWrite(pinBIN1, HIGH);
  digitalWrite(pinBIN2, LOW);
}

void loop() {
  unsigned long waktuSekarang = millis();
  
  if (waktuSekarang - waktuLama >= interval) {
    
    // 1. Kecepatan Aktual = Selisih tick dalam 50ms
    long speedKiri = tickKiri - tickKiriLama;
    long speedKanan = tickKanan - tickKananLama;
    
    tickKiriLama = tickKiri;
    tickKananLama = tickKanan;
    
    // Gunakan nilai mutlak agar kalkulasi konsisten meski mundur
    int aktualSpeedKiri = abs(speedKiri);
    int aktualSpeedKanan = abs(speedKanan);

    // 2. Kalkulasi Error
    errorKiri = targetSpeed - aktualSpeedKiri;
    errorKanan = targetSpeed - aktualSpeedKanan;
    
    // 3. Kalkulasi Integral (dengan sistem Anti-Windup agar tidak kebablasan)
    integralKiri += errorKiri;
    if (integralKiri > 255) integralKiri = 255;
    if (integralKiri < -255) integralKiri = -255;

    integralKanan += errorKanan;
    if (integralKanan > 255) integralKanan = 255;
    if (integralKanan < -255) integralKanan = -255;
    
    // 4. Kalkulasi Derivatif
    float derivatifKiri = errorKiri - lastErrorKiri;
    float derivatifKanan = errorKanan - lastErrorKanan;
    
    // 5. Total Output PID
    float outputKiri = (Kp * errorKiri) + (Ki * integralKiri) + (Kd * derivatifKiri);
    float outputKanan = (Kp * errorKanan) + (Ki * integralKanan) + (Kd * derivatifKanan);
    
    // Simpan error untuk iterasi berikutnya
    lastErrorKiri = errorKiri;
    lastErrorKanan = errorKanan;
    
    // 6. Batasi Output PWM ke jangkauan ESP32 (0 - 255)
    if (outputKiri > 255) outputKiri = 255;
    if (outputKiri < 0) outputKiri = 0;
    
    if (outputKanan > 255) outputKanan = 255;
    if (outputKanan < 0) outputKanan = 0;
    
    // 7. Terapkan PWM ke Motor
    analogWrite(pinPWMA, outputKiri);
    analogWrite(pinPWMB, outputKanan);
    
    waktuLama = waktuSekarang;

    // Tampilkan log
    Serial.print("Raw Kiri:");
    Serial.print(tickKiri);
    Serial.print(", Raw Kanan:");
    Serial.print(tickKanan);
    Serial.print(" | Speed Kiri:");
    Serial.print(aktualSpeedKiri);
    Serial.print(", Speed Kanan:");
    Serial.println(aktualSpeedKanan);
  }
}
