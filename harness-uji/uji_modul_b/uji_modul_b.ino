// ============================================================
//  FIRMWARE UJI MANDIRI - MODUL B (Penggerak & Lokomosi)
//  Fungsi : uji arah motor, uji PWM, hitung encoder, kalibrasi
//  Catatan: firmware ini HANYA untuk pengujian Modul B.
//           Kode final dikerjakan pada Modul D.
//  Board  : ESP32-S3 (Arduino-ESP32 core 2.x atau 3.x)
//  Keamanan: motor berhenti otomatis 3 s setelah perintah gerak
//           terakhir (kirim ulang w/s/a/d/1/2 untuk melanjutkan).
//           Uji pertama dengan RODA DIANGKAT.
// ============================================================

// ---------- Peta pin (sesuaikan bila peta pin berubah) ----------
#define PIN_AIN1    15
#define PIN_AIN2    16
#define PIN_PWMA    17
#define PIN_BIN1    18
#define PIN_BIN2     8
#define PIN_PWMB     9
#define PIN_ENC_L_A  4
#define PIN_ENC_L_B  5
#define PIN_ENC_R_A  6
#define PIN_ENC_R_B  7
// STBY diikat langsung ke 3,3 V - tidak memakai GPIO

#define PWM_FREQ_HZ 20000   // 20 kHz: di atas pendengaran, tidak berdenging
#define TIMEOUT_MOTOR_MS 3000UL   // motor berhenti otomatis bila tidak ada perintah gerak baru

// ---------- Konstanta arah (BELUM DIKONFIRMASI di PCB) ----------
// SIGN_MOT = +1 bila IN1=HIGH/IN2=LOW membuat roda bergerak MAJU robot, -1 bila sebaliknya.
// SIGN_ENC = +1 bila hitungan encoder naik saat roda bergerak MAJU, -1 bila turun.
// Tentukan dengan Bringup_PCB menu 'm' (roda diangkat), lalu salin nilai yang sama ke
// Uji_PID_Kecepatan_Roda_v2. Konvensi hitungan sama: naik bila C1 mendahului C2.
// Nilai sementara = nilai di Uji_PID_Kecepatan_Roda_v2 (encoder kanan negatif saat uji
// breadboard 15 Okt; CN9 di PCB urutannya BO2-BO1 sehingga arah motor kanan bisa berubah).
const int SIGN_MOT_KI = +1, SIGN_MOT_KA = +1;
const int SIGN_ENC_KI = +1, SIGN_ENC_KA = -1;

volatile long tickL = 0;
volatile long tickR = 0;
int  pwmVal = 150;          // 0-255
bool motorAktif = false;
unsigned long tPerintahGerak = 0;   // waktu perintah gerak terakhir (untuk berhenti otomatis)

// ---------- Interrupt encoder (dekode quadrature 2x) ----------
void IRAM_ATTR isrEncL() {
  bool a = digitalRead(PIN_ENC_L_A);
  bool b = digitalRead(PIN_ENC_L_B);
  if (a != b) tickL++; else tickL--;
}

void IRAM_ATTR isrEncR() {
  bool a = digitalRead(PIN_ENC_R_A);
  bool b = digitalRead(PIN_ENC_R_B);
  if (a != b) tickR++; else tickR--;
}

// ---------- Kendali motor dasar ----------
void motorKiri(int arah, int pwm) {       // arah: 1 maju, -1 mundur, 0 berhenti
  arah *= SIGN_MOT_KI;                     // koreksi arah fisik
  digitalWrite(PIN_AIN1, arah > 0);
  digitalWrite(PIN_AIN2, arah < 0);
  analogWrite(PIN_PWMA, arah == 0 ? 0 : pwm);
}

void motorKanan(int arah, int pwm) {
  arah *= SIGN_MOT_KA;
  digitalWrite(PIN_BIN1, arah > 0);
  digitalWrite(PIN_BIN2, arah < 0);
  analogWrite(PIN_PWMB, arah == 0 ? 0 : pwm);
}

void berhenti() {
  motorKiri(0, 0);
  motorKanan(0, 0);
  motorAktif = false;
}

void resetTick() {
  noInterrupts();
  tickL = 0;
  tickR = 0;
  interrupts();
}

void cetakTick() {
  noInterrupts();
  long mL = tickL, mR = tickR;
  interrupts();
  long l = SIGN_ENC_KI * mL, r = SIGN_ENC_KA * mR;   // sudah dikoreksi: positif = maju
  Serial.printf("tickL = %8ld   tickR = %8ld   rata-rata = %8ld   (mentah: %ld / %ld)\n",
                l, r, (l + r) / 2, mL, mR);
}

void tampilkanMenu() {
  Serial.println();
  Serial.println("=========== UJI MODUL B ===========");
  Serial.println("  w = maju          s = mundur");
  Serial.println("  a = putar kiri    d = putar kanan");
  Serial.println("  x = berhenti");
  Serial.println("  + = PWM naik 10   - = PWM turun 10");
  Serial.println("  1 = motor kiri saja");
  Serial.println("  2 = motor kanan saja");
  Serial.println("  r = reset hitungan encoder");
  Serial.println("  p = cetak hitungan encoder");
  Serial.println("  ? = tampilkan menu ini");
  Serial.println("  (motor berhenti otomatis 3 s setelah perintah gerak terakhir)");
  Serial.printf ("  PWM saat ini = %d\n", pwmVal);
  Serial.println("===================================");
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_AIN1, OUTPUT);  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT);  pinMode(PIN_BIN2, OUTPUT);
  pinMode(PIN_PWMA, OUTPUT);  pinMode(PIN_PWMB, OUTPUT);

  // API frekuensi PWM berbeda: core 3.x per pin, core 2.x global
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(PIN_PWMA, PWM_FREQ_HZ);
  analogWriteFrequency(PIN_PWMB, PWM_FREQ_HZ);
#else
  analogWriteFrequency(PWM_FREQ_HZ);
#endif

  pinMode(PIN_ENC_L_A, INPUT_PULLUP);
  pinMode(PIN_ENC_L_B, INPUT_PULLUP);
  pinMode(PIN_ENC_R_A, INPUT_PULLUP);
  pinMode(PIN_ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);

  berhenti();
  tampilkanMenu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c != 0 && strchr("wsad12", c) != NULL) tPerintahGerak = millis();   // perintah gerak baru
    switch (c) {
      case 'w': motorKiri( 1, pwmVal); motorKanan( 1, pwmVal); motorAktif = true;
                Serial.println("-> MAJU");        break;
      case 's': motorKiri(-1, pwmVal); motorKanan(-1, pwmVal); motorAktif = true;
                Serial.println("-> MUNDUR");      break;
      case 'a': motorKiri(-1, pwmVal); motorKanan( 1, pwmVal); motorAktif = true;
                Serial.println("-> PUTAR KIRI");  break;
      case 'd': motorKiri( 1, pwmVal); motorKanan(-1, pwmVal); motorAktif = true;
                Serial.println("-> PUTAR KANAN"); break;
      case '1': motorKiri( 1, pwmVal); motorKanan(0, 0); motorAktif = true;
                Serial.println("-> MOTOR KIRI");  break;
      case '2': motorKiri( 0, 0); motorKanan( 1, pwmVal); motorAktif = true;
                Serial.println("-> MOTOR KANAN"); break;
      case 'x': berhenti(); Serial.println("-> BERHENTI"); break;
      case '+': pwmVal = min(255, pwmVal + 10); Serial.printf("PWM = %d\n", pwmVal); break;
      case '-': pwmVal = max(0,   pwmVal - 10); Serial.printf("PWM = %d\n", pwmVal); break;
      case 'r': resetTick(); Serial.println("-> encoder direset ke 0"); break;
      case 'p': cetakTick(); break;
      case '?': tampilkanMenu(); break;
    }
  }

  // berhenti otomatis bila tidak ada perintah gerak baru selama TIMEOUT_MOTOR_MS
  if (motorAktif && millis() - tPerintahGerak > TIMEOUT_MOTOR_MS) {
    berhenti();
    Serial.println("-> BERHENTI OTOMATIS (3 s tanpa perintah gerak baru)");
    cetakTick();
  }

  // cetak hitungan encoder tiap 500 ms selama motor aktif
  static unsigned long t = 0;
  if (motorAktif && millis() - t > 500) {
    t = millis();
    cetakTick();
  }
}
