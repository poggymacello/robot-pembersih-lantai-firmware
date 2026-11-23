// =====================================================================
//  BRINGUP PCB UTAMA - Robot Pembersih Lantai Otonom (PCB rev 004)
//  Board  : ESP32-S3 DevKit N16R8 (2x22 pin, klon DevKitC-1)
//  Tujuan : satu sketch menu serial untuk menyalakan PCB pertama kali dan
//           tahap integrasi T1, T2, T3 (jog motor), T4, T5, T6, T8
//           (README PCB Utama bagian 9, SPEK_BERSAMA bagian 3).
//  Pustaka: hanya Wire (tanpa pustaka tambahan). Kompatibel core esp32 2.x dan 3.x.
//
//  Pengaturan Arduino IDE (WAJIB):
//    Board ESP32S3 Dev Module, PSRAM "OPI PSRAM", Flash Size 16MB,
//    USB CDC On Boot "Disabled", upload lewat port USB-C "COM" (CH343).
//    GPIO19/20 (USB native) dipakai LED G/B, jadi port USB native tidak dipakai.
//  Serial Monitor: 115200 baud. Ketik satu huruf menu lalu Enter.
//
//  KESELAMATAN:
//    - Daya pertama kali dari catu daya lab dengan batas arus.
//    - Roda DIANGKAT (tidak menyentuh lantai) saat uji motor.
//    - Suction dan motor hanya menyala setelah konfirmasi huruf 'Y' (besar)
//      dan berhenti otomatis. Tombol apa pun saat motor/suction jalan = berhenti.
//
//  Konvensi tanda encoder SAMA dengan Uji_PID_Kecepatan_Roda_v2 dan uji_modul_b:
//    hitungan naik (+) bila C1 mendahului C2. Hasil menu 'm' (SIGN_MOT_*, SIGN_ENC_*)
//    bisa langsung disalin ke kedua sketch tersebut.
// =====================================================================

#include <Wire.h>

#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#error "Atur Tools > USB CDC On Boot = Disabled (GPIO19/20 dipakai LED G/B, upload lewat port COM/CH343)."
#endif

// ---------------- Peta pin (PCB utama rev 004 / SPEK_BERSAMA bagian 3) ----------------
const uint8_t PIN_AIN1 = 15, PIN_AIN2 = 16, PIN_PWMA = 17;   // motor KIRI
const uint8_t PIN_BIN1 = 18, PIN_BIN2 = 8,  PIN_PWMB = 9;    // motor KANAN (STBY diikat 3V3)
const uint8_t ENC_KI_A = 4, ENC_KI_B = 5;                    // encoder kiri C1, C2
const uint8_t ENC_KA_A = 6, ENC_KA_B = 7;                    // encoder kanan C1, C2
const uint8_t PIN_SUCTION = 10;                              // gate MOSFET suction
const uint8_t PIN_VBAT = 1;                                  // ADC1, pembagi 100k/22k lewat JP1
const uint8_t PIN_ECHO_US1 = 11, PIN_ECHO_US4 = 14, PIN_ECHO_US5 = 21;  // lewat pembagi 1k/2k
const uint8_t PIN_XSHUT_KI = 12, PIN_XSHUT_KA = 13;          // VL53L0X TOF-KI / TOF-KA
const uint8_t PIN_TCRT_KI = 41, PIN_TCRT_KA = 42;
const uint8_t PIN_SDA = 47, PIN_SCL = 48;
const uint8_t PIN_CAM_TX = 39, PIN_CAM_RX = 40;              // UART ke ESP32-S3-CAM (tidak diuji di sini)
const uint8_t PIN_LED_R = 38, PIN_LED_G = 19, PIN_LED_B = 20; // RGB katoda bersama, HIGH = nyala
const uint8_t PIN_BUZZER = 2;                                // lewat NPN, HIGH = bunyi
// GPIO 35, 36, 37 = PSRAM octal: JANGAN dipakai.

// ---------------- Alamat I2C ----------------
const uint8_t ADDR_PCF = 0x20;
const uint8_t ADDR_TOF_DEFAULT = 0x29;
const uint8_t ADDR_TOF_KI = 0x30, ADDR_TOF_KA = 0x31;
const uint8_t ADDR_EEPROM = 0x57;
const uint8_t ADDR_RTC = 0x68;
const uint8_t ADDR_MPU = 0x69;
const uint8_t ADDR_AK8963 = 0x0C;   // magnetometer, hanya terlihat bila mode bypass aktif

// Bit TRIG HC-SR04 di PCF8574
const uint8_t PCF_TRIG_US1 = 0, PCF_TRIG_US4 = 3, PCF_TRIG_US5 = 4;

// ---------------- Parameter uji ----------------
const uint32_t I2C_HZ = 100000;      // PCF8574 (versi non-A) maksimum 100 kHz
const uint32_t PWM_FREQ_HZ = 20000;  // PWM motor 20 kHz
const int PWM_JOG = 80;              // PWM rendah untuk jog motor (0-255)
const unsigned long JOG_MS = 1000;   // lama jog tiap arah
const unsigned long SUCTION_MS = 3000;
const unsigned long ECHO_TIMEOUT_US = 30000;  // ~5 m
const float VBAT_RASIO = 122.0f / 22.0f;      // (100k + 22k) / 22k

// ---------------- State ----------------
volatile long cntKi = 0, cntKa = 0;   // hitungan 4x
volatile uint8_t stKi = 0, stKa = 0;
bool pcfAda = false;
bool tofSudahDiatur = false;

// Tabel transisi kuadratur (sama dengan Uji_PID_Kecepatan_Roda_v2): indeks = (lama << 2) | baru
const int8_t QDEC[16] = {0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0};

void IRAM_ATTR isrKi() {
  uint8_t s = (digitalRead(ENC_KI_A) << 1) | digitalRead(ENC_KI_B);
  cntKi += QDEC[(stKi << 2) | s];
  stKi = s;
}
void IRAM_ATTR isrKa() {
  uint8_t s = (digitalRead(ENC_KA_A) << 1) | digitalRead(ENC_KA_B);
  cntKa += QDEC[(stKa << 2) | s];
  stKa = s;
}

// =====================================================================
//  Utilitas
// =====================================================================
void garis() { Serial.println(F("--------------------------------------------------------------")); }

// Buang sisa karakter di buffer serial (mis. CR/LF dari Serial Monitor)
void kurasSerial() {
  delay(20);
  while (Serial.available()) { Serial.read(); delay(2); }
}

// Tunggu satu karakter (abaikan CR, LF, spasi). Kembalikan 0 bila timeout.
char tungguTombol(unsigned long timeoutMs) {
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c != '\r' && c != '\n' && c != ' ') return c;
    }
    delay(5);
  }
  return 0;
}

// Ada tombol ditekan (untuk menghentikan uji)? Abaikan CR/LF.
bool adaTombol() {
  while (Serial.available()) {
    char c = Serial.peek();
    if (c == '\r' || c == '\n' || c == ' ') { Serial.read(); continue; }
    return true;
  }
  return false;
}

bool konfirmasiY(const __FlashStringHelper *pesan) {
  kurasSerial();
  Serial.println(pesan);
  Serial.println(F("Ketik Y (huruf besar) lalu Enter untuk lanjut, tombol lain = batal (batas 20 s)."));
  char c = tungguTombol(20000);
  kurasSerial();
  if (c != 'Y') { Serial.println(F("-> DIBATALKAN.")); return false; }
  return true;
}

// ---------------- I2C dasar ----------------
bool i2cAda(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cTulisReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

// Baca n byte mulai dari reg. Kembalikan true bila semua byte diterima.
bool i2cBacaReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;   // pakai STOP, aman untuk semua perangkat
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

int i2cBaca1(uint8_t addr, uint8_t reg) {   // -1 bila gagal
  uint8_t v;
  return i2cBacaReg(addr, reg, &v, 1) ? v : -1;
}

bool pcfTulis(uint8_t val) {
  Wire.beginTransmission(ADDR_PCF);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int pcfBaca() {
  if (Wire.requestFrom(ADDR_PCF, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// ---------------- Motor (tanpa tanda; +cmd = IN1 HIGH, IN2 LOW) ----------------
void motorMentah(uint8_t in1, uint8_t in2, uint8_t pwmPin, int cmd) {
  cmd = constrain(cmd, -255, 255);
  if (cmd > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (cmd < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else              { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }  // berhenti (coast)
  analogWrite(pwmPin, abs(cmd));
}

void motorBerhentiSemua() {
  motorMentah(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0);
  motorMentah(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0);
}

void bacaEncoder(long &ki, long &ka) {
  noInterrupts(); ki = cntKi; ka = cntKa; interrupts();
}

void resetEncoder() {
  noInterrupts(); cntKi = 0; cntKa = 0; interrupts();
}

// ---------------- Keadaan aman ----------------
// Dipanggil paling awal di setup(): semua aktuator mati, kedua VL53L0X dimatikan (XSHUT LOW).
void keadaanAman() {
  const uint8_t keluaranLow[] = {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB,
                                 PIN_SUCTION, PIN_XSHUT_KI, PIN_XSHUT_KA, PIN_BUZZER,
                                 PIN_LED_R, PIN_LED_G, PIN_LED_B};
  for (uint8_t p : keluaranLow) {
    pinMode(p, OUTPUT);     // register output bawaan reset = 0, jadi langsung LOW
    digitalWrite(p, LOW);
  }
}

// =====================================================================
//  Menu 'i' - Info chip (T2 Mikrokontroler)
// =====================================================================
void menuInfoChip() {
  garis();
  Serial.println(F("[T2] Info chip ESP32-S3"));
  Serial.printf("Model chip   : %s rev %d, %d inti, CPU %lu MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                (unsigned long)ESP.getCpuFreqMHz());
  Serial.printf("Flash        : %lu MB\n", (unsigned long)(ESP.getFlashChipSize() / (1024UL * 1024UL)));
  Serial.printf("PSRAM        : %lu KB (harus ~8192 KB untuk N16R8)\n", (unsigned long)(ESP.getPsramSize() / 1024UL));
  Serial.printf("Heap bebas   : %lu B\n", (unsigned long)ESP.getFreeHeap());
#if defined(ESP_ARDUINO_VERSION_MAJOR)
  Serial.printf("Core Arduino : %d.%d.%d\n", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
#endif
  if (ESP.getFlashChipSize() < 16UL * 1024UL * 1024UL)
    Serial.println(F("CATATAN: Flash terbaca < 16 MB -> cek Tools > Flash Size = 16MB."));
  if (ESP.getPsramSize() == 0)
    Serial.println(F("CATATAN: PSRAM 0 -> cek Tools > PSRAM = OPI PSRAM (N16R8 memakai PSRAM octal)."));
}

// =====================================================================
//  Menu 's' - Pemindai I2C dengan PASS/FAIL (T2, T5, T6)
// =====================================================================
struct PerangkatI2C { uint8_t addr; const char *nama; const char *saran; };
const PerangkatI2C DIHARAPKAN[] = {
  {0x20, "PCF8574 (U3)",            "cek IC U3 terpasang dan orientasinya, A0-A2 ke GND, 3V3 di pin 16"},
  {0x30, "VL53L0X TOF-KI (CN12)",   "jalankan menu 't' (urutan alamat ToF) lalu pindai lagi"},
  {0x31, "VL53L0X TOF-KA (CN13)",   "jalankan menu 't' (urutan alamat ToF) lalu pindai lagi"},
  {0x57, "EEPROM modul DS3231",     "cek modul DS3231 di CN16 (sebagian modul tanpa EEPROM)"},
  {0x68, "DS3231 RTC (CN16)",       "cek kabel CN16: 3V3, GND, SCL, SDA"},
  {0x69, "MPU9250 (U4)",            "cek solderan U4, AD0 harus ke 3V3 (0x68 bila AD0 ke GND)"},
};
const uint8_t N_DIHARAPKAN = sizeof(DIHARAPKAN) / sizeof(DIHARAPKAN[0]);

bool menuScanI2C() {
  garis();
  Serial.println(F("[I2C] Pemindai bus SDA=47 SCL=48"));
  bool ada[128] = {false};
  int jumlah = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cAda(a)) { ada[a] = true; jumlah++; Serial.printf("  ditemukan 0x%02X\n", a); }
  }
  if (jumlah == 0) {
    Serial.println(F("FAIL: tidak ada perangkat. Cek pull-up 4,7k, 3V3, SDA/SCL tertukar, atau bus terhubung singkat."));
    return false;
  }

  bool lulus = true;
  Serial.println(F("Perbandingan dengan daftar yang diharapkan (0x20, 0x30, 0x31, 0x57, 0x68, 0x69):"));
  for (uint8_t i = 0; i < N_DIHARAPKAN; i++) {
    const PerangkatI2C &p = DIHARAPKAN[i];
    if (ada[p.addr]) Serial.printf("  OK     0x%02X %s\n", p.addr, p.nama);
    else { Serial.printf("  HILANG 0x%02X %s -> %s\n", p.addr, p.nama, p.saran); lulus = false; }
  }
  // perangkat tak terduga
  for (uint8_t a = 1; a < 127; a++) {
    if (!ada[a]) continue;
    bool dikenal = false;
    for (uint8_t i = 0; i < N_DIHARAPKAN; i++) if (DIHARAPKAN[i].addr == a) dikenal = true;
    if (dikenal) continue;
    if (a == ADDR_AK8963) {
      Serial.println(F("  INFO   0x0C AK8963 (magnetometer MPU9250, mode bypass aktif) - normal."));
    } else if (a == ADDR_TOF_DEFAULT) {
      lulus = false;
      Serial.println(F("  PERHATIAN 0x29 = VL53L0X masih di alamat bawaan. Artinya:"));
      Serial.println(F("    - urutan alamat belum dijalankan / gagal, atau"));
      Serial.println(F("    - XSHUT tidak bisa mematikan sensor (pull-up di modul melawan pull-down 10k:"));
      Serial.println(F("      ganti R9/R10 jadi 2,2k), atau kabel XSHUT (pin 5 CN12/CN13) salah."));
      Serial.println(F("    Bila KEDUA sensor menyala dengan XSHUT HIGH tanpa urutan, keduanya bentrok di 0x29."));
      Serial.println(F("    Jalankan menu 't' lalu pindai lagi."));
    } else {
      Serial.printf("  TAK DIKENAL 0x%02X (perangkat lain / alamat salah / gangguan bus)\n", a);
      lulus = false;
    }
  }
  if (!tofSudahDiatur && !ada[ADDR_TOF_KI] && !ada[ADDR_TOF_KA])
    Serial.println(F("Catatan: urutan ToF belum dijalankan sejak boot (XSHUT LOW), jadi 0x30/0x31 memang belum ada."));
  Serial.println(lulus ? F("HASIL I2C: PASS") : F("HASIL I2C: FAIL / BELUM LENGKAP"));
  return lulus;
}

// =====================================================================
//  Menu 't' - Urutan alamat VL53L0X + cek model ID (T5)
// =====================================================================
void cetakIdTof(uint8_t addr) {
  int idModel = i2cBaca1(addr, 0xC0);
  int idTipe  = i2cBaca1(addr, 0xC1);
  int idRev   = i2cBaca1(addr, 0xC2);
  Serial.printf("    reg 0xC0 (model ID) = 0x%02X %s, 0xC1 = 0x%02X (harap 0xAA), 0xC2 (revisi) = 0x%02X\n",
                idModel & 0xFF, idModel == 0xEE ? "(OK, VL53L0X)" : "(SALAH, harap 0xEE)",
                idTipe & 0xFF, idRev & 0xFF);
}

bool lepasDanAturTof(uint8_t pinXshut, uint8_t alamatBaru, const char *nama) {
  Serial.printf("  %s: XSHUT GPIO%d = HIGH (output), tunggu 10 ms...\n", nama, pinXshut);
  digitalWrite(pinXshut, HIGH);   // tetap OUTPUT; jangan jadikan input (pull-down 10k menariknya LOW)
  delay(10);
  if (!i2cAda(ADDR_TOF_DEFAULT)) {
    Serial.printf("  FAIL %s: tidak menjawab di 0x29. Cek kabel CN (3V3, GND, SCL, SDA, XSHUT), modul dicatu 3V3.\n", nama);
    return false;
  }
  int id = i2cBaca1(ADDR_TOF_DEFAULT, 0xC0);
  Serial.printf("  %s menjawab di 0x29, model ID 0x%02X %s\n", nama, id & 0xFF, id == 0xEE ? "(OK)" : "(SALAH, harap 0xEE)");
  // Register 0x8A I2C_SLAVE_DEVICE_ADDRESS: tulis alamat 7-bit
  if (!i2cTulisReg(ADDR_TOF_DEFAULT, 0x8A, alamatBaru & 0x7F)) {
    Serial.printf("  FAIL %s: tulis register 0x8A gagal.\n", nama);
    return false;
  }
  delay(2);
  if (!i2cAda(alamatBaru)) {
    Serial.printf("  FAIL %s: tidak menjawab di alamat baru 0x%02X.\n", nama, alamatBaru);
    return false;
  }
  Serial.printf("  OK %s sekarang di 0x%02X\n", nama, alamatBaru);
  cetakIdTof(alamatBaru);
  return i2cBaca1(alamatBaru, 0xC0) == 0xEE;
}

void menuUrutanTof() {
  garis();
  Serial.println(F("[T5] Urutan alamat VL53L0X (TOF-KI -> 0x30, TOF-KA -> 0x31)"));
  digitalWrite(PIN_XSHUT_KI, LOW);
  digitalWrite(PIN_XSHUT_KA, LOW);
  delay(10);
  Serial.println(F("  Langkah 1: kedua XSHUT LOW (sensor mati). Ukur pin 5 CN12/CN13: ~0 V."));
  if (i2cAda(ADDR_TOF_DEFAULT) || i2cAda(ADDR_TOF_KI) || i2cAda(ADDR_TOF_KA)) {
    Serial.println(F("  PERHATIAN: masih ada VL53L0X yang menjawab saat XSHUT LOW."));
    Serial.println(F("    Kemungkinan pull-up XSHUT di modul melawan pull-down 10k (ganti R9/R10 = 2,2k)"));
    Serial.println(F("    atau kabel XSHUT tidak tersambung ke pin XSHUT modul. Urutan tetap dicoba."));
  }
  bool okKi = lepasDanAturTof(PIN_XSHUT_KI, ADDR_TOF_KI, "TOF-KI");
  bool okKa = lepasDanAturTof(PIN_XSHUT_KA, ADDR_TOF_KA, "TOF-KA");
  tofSudahDiatur = true;
  if (i2cAda(ADDR_TOF_DEFAULT))
    Serial.println(F("  PERHATIAN: masih ada perangkat di 0x29 setelah urutan (lihat penjelasan menu 's')."));
  Serial.println(F("  Ukur pin 5 CN12/CN13 sekarang: ~3 V (XSHUT HIGH)."));
  Serial.println(F("  Catatan: pengukuran jarak memerlukan inisialisasi lengkap (mis. pustaka VL53L0X) di sketch berikutnya."));
  Serial.println((okKi && okKa) ? F("HASIL ToF: PASS") : F("HASIL ToF: FAIL"));
}

// =====================================================================
//  Menu 'g' - MPU9250 WHO_AM_I + satu bacaan (T6)
// =====================================================================
void menuImu() {
  garis();
  Serial.println(F("[T6] IMU MPU9250 di 0x69"));
  int who = i2cBaca1(ADDR_MPU, 0x75);
  if (who < 0) {
    Serial.println(F("FAIL: tidak ada jawaban di 0x69. Cek solderan U4, AD0 ke 3V3, NCS ke 3V3."));
    if (i2cAda(0x68)) Serial.println(F("  (0x68 menjawab, tetapi itu kemungkinan DS3231; bila RTC dicabut dan 0x68 tetap ada, AD0 IMU = GND)"));
    return;
  }
  const char *nama = "TIDAK DIKENAL";
  if (who == 0x71) nama = "MPU9250";
  else if (who == 0x73) nama = "MPU9255";
  else if (who == 0x70) nama = "MPU6500 (TANPA magnetometer)";
  Serial.printf("WHO_AM_I (0x75) = 0x%02X -> %s\n", who, nama);
  // bangunkan sensor, baca akselerometer/suhu/gyro mentah (skala bawaan +-2 g, +-250 dps)
  i2cTulisReg(ADDR_MPU, 0x6B, 0x00);
  delay(50);
  uint8_t b[14];
  if (i2cBacaReg(ADDR_MPU, 0x3B, b, 14)) {
    int16_t ax = (b[0] << 8) | b[1], ay = (b[2] << 8) | b[3], az = (b[4] << 8) | b[5];
    int16_t tr = (b[6] << 8) | b[7];
    int16_t gx = (b[8] << 8) | b[9], gy = (b[10] << 8) | b[11], gz = (b[12] << 8) | b[13];
    Serial.printf("Akselerometer [g]  : x=%+.3f y=%+.3f z=%+.3f (diam datar: |z| ~ 1)\n",
                  ax / 16384.0f, ay / 16384.0f, az / 16384.0f);
    Serial.printf("Gyro [dps]         : x=%+.2f y=%+.2f z=%+.2f (diam: dekat 0)\n",
                  gx / 131.0f, gy / 131.0f, gz / 131.0f);
    Serial.printf("Suhu chip [C]      : %.1f\n", tr / 333.87f + 21.0f);
  } else {
    Serial.println(F("Gagal membaca data sensor."));
  }
  Serial.println((who == 0x71 || who == 0x73) ? F("HASIL IMU: PASS") :
                 (who == 0x70 ? F("HASIL IMU: PERHATIAN (chip MPU6500, magnetometer tidak ada)") : F("HASIL IMU: FAIL")));
  Serial.println(F("Kalibrasi lengkap: sketch Kalibrasi_IMU."));
}

// =====================================================================
//  Menu 'r' / 'T' - DS3231 baca / set waktu (T6)
// =====================================================================
uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

void menuRtcBaca() {
  garis();
  Serial.println(F("[T6] RTC DS3231 di 0x68"));
  uint8_t r[7];
  if (!i2cBacaReg(ADDR_RTC, 0x00, r, 7)) {
    Serial.println(F("FAIL: DS3231 tidak menjawab. Cek CN16 (3V3, GND, SCL, SDA)."));
    return;
  }
  uint8_t detik = bcd2dec(r[0] & 0x7F), menit = bcd2dec(r[1] & 0x7F);
  uint8_t jam;
  if (r[2] & 0x40) {   // mode 12 jam
    jam = bcd2dec(r[2] & 0x1F) % 12 + ((r[2] & 0x20) ? 12 : 0);
  } else {
    jam = bcd2dec(r[2] & 0x3F);
  }
  uint8_t tgl = bcd2dec(r[4] & 0x3F), bln = bcd2dec(r[5] & 0x1F);
  uint16_t thn = 2000 + bcd2dec(r[6]) + ((r[5] & 0x80) ? 100 : 0);
  const char *HARI[8] = {"?", "Minggu", "Senin", "Selasa", "Rabu", "Kamis", "Jumat", "Sabtu"};
  uint8_t h = r[3] & 0x07;
  Serial.printf("Waktu RTC: %s %02u-%02u-%04u %02u:%02u:%02u\n", HARI[h <= 7 ? h : 0], tgl, bln, thn, jam, menit, detik);
  int status = i2cBaca1(ADDR_RTC, 0x0F);
  if (status >= 0 && (status & 0x80))
    Serial.println(F("PERHATIAN: flag OSF=1 (osilator pernah berhenti / baterai RTC habis) -> waktu tidak valid, set dengan menu 'T'."));
  uint8_t t[2];
  if (i2cBacaReg(ADDR_RTC, 0x11, t, 2)) {
    float suhu = (int8_t)t[0] + (t[1] >> 6) * 0.25f;
    Serial.printf("Suhu DS3231: %.2f C\n", suhu);
  }
  Serial.println(i2cAda(ADDR_EEPROM) ? F("EEPROM 0x57: ada") : F("EEPROM 0x57: tidak menjawab"));
  Serial.println(F("Jalankan lagi setelah beberapa detik: detik harus bertambah."));
  Serial.println(F("HASIL RTC: PASS (bila waktu berjalan dan OSF=0)"));
}

// Set RTC ke waktu kompilasi sketch (__DATE__ "Mmm dd yyyy", __TIME__ "hh:mm:ss")
void menuRtcSet() {
  garis();
  const char *BULAN = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char nb[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
  int bln = (strstr(BULAN, nb) - BULAN) / 3 + 1;
  int tgl = atoi(__DATE__ + 4);
  int thn = atoi(__DATE__ + 7);
  int jam = atoi(__TIME__), mnt = atoi(__TIME__ + 3), dtk = atoi(__TIME__ + 6);
  // hari dalam minggu (Sakamoto): 0 = Minggu
  static const int tt[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  int y = thn - (bln < 3);
  int hari = (y + y / 4 - y / 100 + y / 400 + tt[bln - 1] + tgl) % 7;
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x00);
  Wire.write(dec2bcd(dtk));
  Wire.write(dec2bcd(mnt));
  Wire.write(dec2bcd(jam));        // mode 24 jam
  Wire.write(hari + 1);            // 1 = Minggu
  Wire.write(dec2bcd(tgl));
  Wire.write(dec2bcd(bln));        // bit abad = 0 (tahun 2000-2099)
  Wire.write(dec2bcd(thn % 100));
  bool ok = Wire.endTransmission() == 0;
  int status = i2cBaca1(ADDR_RTC, 0x0F);
  if (status >= 0) i2cTulisReg(ADDR_RTC, 0x0F, status & ~0x80);   // hapus flag OSF
  Serial.printf("[T6] RTC diset ke waktu kompilasi %s %s: %s\n", __DATE__, __TIME__, ok ? "OK" : "GAGAL");
  Serial.println(F("Catatan: waktu kompilasi tertinggal beberapa puluh detik dari waktu nyata."));
}

// =====================================================================
//  Menu 'x' - Uji tulis PCF8574 (T5)
// =====================================================================
void menuPcf() {
  garis();
  Serial.println(F("[T5] Uji tulis/baca PCF8574 (0x20)"));
  if (!i2cAda(ADDR_PCF)) { Serial.println(F("FAIL: PCF8574 tidak menjawab.")); pcfAda = false; return; }
  pcfAda = true;
  const uint8_t pola[] = {0x00, 0xFF, 0x55, 0xAA, 0x00};
  bool lulus = true;
  for (uint8_t p : pola) {
    pcfTulis(p);
    delay(5);
    int baca = pcfBaca();
    bool ok = (baca == p);
    if (!ok) lulus = false;
    Serial.printf("  tulis 0x%02X baca 0x%02X %s\n", p, baca & 0xFF, ok ? "OK" : "BEDA");
  }
  Serial.println(F("  Tahan semua pin HIGH 3 s: ukur U3 pin 4 (P0), 7 (P3), 9 (P4) ~3,3 V..."));
  pcfTulis(0xFF); delay(3000);
  Serial.println(F("  Tahan semua pin LOW 3 s: ukur pin yang sama ~0 V..."));
  pcfTulis(0x00); delay(3000);
  pcfTulis(0x00);   // keadaan diam: semua TRIG LOW
  if (!lulus) Serial.println(F("  Bit yang BEDA: pin tertarik LOW dari luar (hubung singkat / beban) atau IC rusak."));
  Serial.println(lulus ? F("HASIL PCF8574: PASS") : F("HASIL PCF8574: FAIL"));
}

// =====================================================================
//  Menu 'u' - HC-SR04 (TRIG lewat PCF8574, ECHO pulseIn) (T5)
// =====================================================================
float ukurHcsr04(uint8_t bitTrig, uint8_t pinEcho, bool &echoMacet) {
  echoMacet = digitalRead(pinEcho) == HIGH;
  if (echoMacet) return -1;
  pcfTulis(0x00);
  delayMicroseconds(4);
  pcfTulis(1 << bitTrig);   // TRIG HIGH (transaksi I2C 100 kHz sendiri sudah > 10 us)
  delayMicroseconds(10);
  pcfTulis(0x00);           // TRIG LOW -> sensor mengirim burst
  unsigned long us = pulseIn(pinEcho, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1;   // timeout
  return us / 58.0f;        // cm
}

void menuHcsr04() {
  garis();
  Serial.println(F("[T5] HC-SR04: US1 depan (P0/GPIO11), US4 kiri 90 (P3/GPIO14), US5 kanan 90 (P4/GPIO21)"));
  if (!i2cAda(ADDR_PCF)) { Serial.println(F("FAIL: PCF8574 tidak menjawab, TRIG tidak bisa dikirim.")); return; }
  struct { const char *nama; uint8_t bit; uint8_t echo; } us[3] = {
    {"US1 depan  ", PCF_TRIG_US1, PIN_ECHO_US1},
    {"US4 kiri90 ", PCF_TRIG_US4, PIN_ECHO_US4},
    {"US5 kanan90", PCF_TRIG_US5, PIN_ECHO_US5},
  };
  for (uint8_t i = 0; i < 3; i++) {
    Serial.printf("  %s:", us[i].nama);
    int sukses = 0;
    bool macet = false;
    for (uint8_t k = 0; k < 5; k++) {
      bool m;
      float cm = ukurHcsr04(us[i].bit, us[i].echo, m);
      if (m) macet = true;
      if (cm < 0) Serial.print(F("   ---"));
      else { Serial.printf(" %6.1f", cm); sukses++; }
      delay(60);   // jeda antar ukur agar gema lama hilang
    }
    Serial.print(F(" cm"));
    if (macet) Serial.print(F("  <- ECHO macet HIGH (cek pembagi 1k/2k / kabel)"));
    else if (sukses == 0) Serial.print(F("  <- tidak ada echo (cek 5V, TRIG, ECHO, objek terlalu jauh)"));
    Serial.println();
  }
  pcfTulis(0x00);
  Serial.println(F("Letakkan benda pada jarak diketahui (mis. 20 cm) di depan tiap sensor; '---' = timeout."));
}

// =====================================================================
//  Menu 'l' - TCRT5000 (T6)
// =====================================================================
void menuTcrt() {
  garis();
  Serial.println(F("[T6] TCRT5000 kiri GPIO41, kanan GPIO42 selama 10 s (tombol apa pun = berhenti)"));
  Serial.println(F("  Modul umum: LOW = ada pantulan (lantai dekat), HIGH = tidak ada pantulan (tepi/tebing)."));
  Serial.println(F("  Cocokkan dengan LED di modul; atur potensiometer modul bila perlu."));
  kurasSerial();
  unsigned long t0 = millis();
  while (millis() - t0 < 10000 && !adaTombol()) {
    int a = digitalRead(PIN_TCRT_KI), b = digitalRead(PIN_TCRT_KA);
    Serial.printf("  KI=%d (%s)   KA=%d (%s)\n", a, a ? "tak ada pantulan" : "pantulan",
                  b, b ? "tak ada pantulan" : "pantulan");
    delay(250);
  }
  kurasSerial();
}

// =====================================================================
//  Menu 'b' - Tegangan baterai (T1)
// =====================================================================
uint32_t bacaVadcMv() {
  uint32_t jml = 0;
  const int N = 64;
  for (int i = 0; i < N; i++) { jml += analogReadMilliVolts(PIN_VBAT); delayMicroseconds(200); }
  return jml / N;
}

void menuVbat() {
  garis();
  Serial.println(F("[T1] Sense baterai GPIO1 (pembagi 100k/22k lewat JP1)"));
  uint32_t mv = bacaVadcMv();
  float vbat = mv * VBAT_RASIO / 1000.0f;
  Serial.printf("  Vadc = %lu mV   VBAT = Vadc x 122/22 = %.2f V   (mentah analogRead = %d)\n",
                (unsigned long)mv, vbat, analogRead(PIN_VBAT));
  if (mv > 3200) Serial.println(F("  PERINGATAN: Vadc > 3,2 V! Batas aman pin 3,3 V. Cek nilai resistor pembagi / VBAT terlalu tinggi."));
  else if (mv > 3050) Serial.println(F("  Catatan: Vadc > ~3,05 V mendekati batas ukur ADC (11 dB), hasil bisa jenuh."));
  if (mv < 50) Serial.println(F("  Vadc ~0: JP1 belum dipasang, CN2 (VBAT) belum disambung, atau R pembagi terbuka."));
  else if (vbat < 12.8f) Serial.println(F("  Baterai rendah (< 3,2 V/sel). Isi ulang sebelum uji motor."));
  else if (vbat > 17.0f) Serial.println(F("  VBAT > 17 V: tidak wajar untuk 4S (maks 16,8 V). Cek pembagi / sumber."));
  Serial.println(F("  Bandingkan dengan multimeter di TP1; selisih > 3% -> koreksi faktor VBAT_RASIO."));
}

// =====================================================================
//  Menu 'c' - LED RGB, 'z' - buzzer (T8)
// =====================================================================
void setLed(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, r); digitalWrite(PIN_LED_G, g); digitalWrite(PIN_LED_B, b);
}

void menuLed() {
  garis();
  Serial.println(F("[T8] Siklus LED RGB (katoda bersama, HIGH = nyala)"));
  struct { const char *nama; bool r, g, b; } w[] = {
    {"MERAH", 1, 0, 0}, {"HIJAU", 0, 1, 0}, {"BIRU", 0, 0, 1},
    {"KUNING", 1, 1, 0}, {"CYAN", 0, 1, 1}, {"MAGENTA", 1, 0, 1}, {"PUTIH", 1, 1, 1},
  };
  for (auto &x : w) {
    Serial.printf("  %s\n", x.nama);
    setLed(x.r, x.g, x.b);
    delay(800);
  }
  setLed(0, 0, 0);
  Serial.println(F("  MATI. Warna tertukar -> cek urutan kabel CN21 (R=GPIO38, G=GPIO19, B=GPIO20)."));
  Serial.println(F("  Semua terbalik (nyala saat seharusnya mati) -> LED anoda bersama, balik logika."));
}

void menuBuzzer() {
  garis();
  Serial.println(F("[T8] Buzzer GPIO2: 3 kali bunyi"));
  for (int i = 0; i < 3; i++) {
    digitalWrite(PIN_BUZZER, HIGH); delay(150);
    digitalWrite(PIN_BUZZER, LOW);  delay(150);
  }
  Serial.println(F("  Tidak bunyi -> cek buzzer AKTIF 5V di CN10 (polaritas: pin 1 +5V, pin 2 BUZ-) dan transistor Q2."));
}

// =====================================================================
//  Menu 'v' - Suction 3 s dengan konfirmasi (T4)
// =====================================================================
void menuSuction() {
  garis();
  Serial.println(F("[T4] Uji motor suction (GPIO10 -> MOSFET Q1), 3 s"));
  if (!konfirmasiY(F("Pastikan impeller bebas, tidak ada benda/jari di saluran hisap, CN4 dan CN3 (12V-A) tersambung."))) return;
  Serial.println(F("  SUCTION NYALA (tombol apa pun = berhenti)"));
  uint32_t mvSebelum = bacaVadcMv();
  digitalWrite(PIN_SUCTION, HIGH);
  unsigned long t0 = millis();
  bool dibatalkan = false;
  uint32_t mvSaatNyala = 0;
  while (millis() - t0 < SUCTION_MS) {
    if (adaTombol()) { dibatalkan = true; break; }
    if (mvSaatNyala == 0 && millis() - t0 > 1500) mvSaatNyala = bacaVadcMv();
    delay(10);
  }
  digitalWrite(PIN_SUCTION, LOW);
  kurasSerial();
  Serial.println(dibatalkan ? F("  SUCTION MATI (dihentikan pengguna)") : F("  SUCTION MATI (3 s selesai)"));
  if (mvSaatNyala > 0)
    Serial.printf("  VBAT sebelum = %.2f V, saat nyala = %.2f V (turun %.2f V)\n",
                  mvSebelum * VBAT_RASIO / 1000.0f, mvSaatNyala * VBAT_RASIO / 1000.0f,
                  (float)((int32_t)mvSebelum - (int32_t)mvSaatNyala) * VBAT_RASIO / 1000.0f);
  Serial.println(F("  Saat HIGH: TP9 ~3,3 V dan motor berputar. Tidak berputar -> cek Q1, CN4, rel 12V-A."));
}

// =====================================================================
//  Menu 'm' - Jog motor + penentuan konstanta tanda (T3)
// =====================================================================
// Jalankan satu motor dengan perintah mentah cmd selama JOG_MS. Kembalikan hitungan 4x
// (bertanda, tanpa koreksi). abort = true bila pengguna menekan tombol.
long jogSatuArah(bool kiri, int cmd, bool &abort) {
  resetEncoder();
  if (kiri) motorMentah(PIN_AIN1, PIN_AIN2, PIN_PWMA, cmd);
  else      motorMentah(PIN_BIN1, PIN_BIN2, PIN_PWMB, cmd);
  unsigned long t0 = millis();
  abort = false;
  while (millis() - t0 < JOG_MS) {
    if (adaTombol()) { abort = true; break; }
    delay(5);
  }
  motorBerhentiSemua();   // berhenti otomatis
  delay(400);             // biarkan roda berhenti, hitungan ikut terekam
  long ki, ka;
  bacaEncoder(ki, ka);
  return kiri ? ki : ka;
}

void jogMotor(bool kiri) {
  const char *nama = kiri ? "KIRI" : "KANAN";
  const char *pinIn = kiri ? "AIN1=HIGH AIN2=LOW" : "BIN1=HIGH BIN2=LOW";
  long hitLain;
  Serial.printf("\n  Motor %s: perintah + (%s), PWM %d, %lu ms\n", nama, pinIn, PWM_JOG, JOG_MS);
  bool abort;
  long cPlus = jogSatuArah(kiri, +PWM_JOG, abort);
  { long a, b; bacaEncoder(a, b); hitLain = kiri ? b : a; }
  Serial.printf("    hitungan 4x encoder %s = %+ld (1x ~ %+ld)   [encoder sisi lain = %+ld, harus ~0]\n",
                nama, cPlus, cPlus / 4, hitLain);
  if (abort) { kurasSerial(); Serial.println(F("  DIHENTIKAN pengguna.")); return; }

  kurasSerial();
  Serial.printf("    Apakah roda %s tadi berputar ke arah MAJU robot? ketik y / n (lain = lewati)\n", nama);
  char jawab = tungguTombol(30000);
  kurasSerial();

  Serial.printf("  Motor %s: perintah - (kebalikan), PWM %d, %lu ms\n", nama, PWM_JOG, JOG_MS);
  long cMinus = jogSatuArah(kiri, -PWM_JOG, abort);
  Serial.printf("    hitungan 4x encoder %s = %+ld (1x ~ %+ld)\n", nama, cMinus, cMinus / 4);
  if (abort) { kurasSerial(); Serial.println(F("  DIHENTIKAN pengguna.")); return; }

  // Pemeriksaan kewajaran
  if (labs(cPlus) < 20 && labs(cMinus) < 20) {
    Serial.println(F("    MASALAH: encoder hampir tidak berubah. Motor tidak berputar (cek CN6/CN9, rel 12V-B, TB6612, PWM)"));
    Serial.println(F("    atau encoder tidak terbaca (cek CN7/CN8, VCC encoder 3V3). Coba menu 'e' sambil memutar roda."));
    return;
  }
  if ((cPlus > 0) == (cMinus > 0))
    Serial.println(F("    MASALAH: tanda hitungan sama di kedua arah -> salah satu kanal C1/C2 tidak terbaca (dekode rusak)."));

  if (jawab != 'y' && jawab != 'n') {
    Serial.println(F("    Arah fisik tidak dijawab: konstanta tanda tidak dihitung. Ulangi menu 'm'."));
    return;
  }
  int signMot = (jawab == 'y') ? +1 : -1;
  long cMaju = (jawab == 'y') ? cPlus : cMinus;   // hitungan saat roda bergerak maju
  int signEnc = (cMaju >= 0) ? +1 : -1;
  const char *suf = kiri ? "KI" : "KA";
  Serial.printf("  ==> SIGN_MOT_%s = %+d   SIGN_ENC_%s = %+d\n", suf, signMot, suf, signEnc);
  Serial.println(F("      (salin ke uji_modul_b dan Uji_PID_Kecepatan_Roda_v2, lalu cek: perintah maju -> tick naik +)"));
}

void menuMotor() {
  garis();
  Serial.println(F("[T3] Jog motor: tiap motor maju lalu mundur, PWM rendah, berhenti otomatis."));
  if (!konfirmasiY(F("RODA HARUS DIANGKAT (tidak menyentuh lantai). Rel 12V-B dan TB6612 terpasang."))) return;
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);
  jogMotor(true);
  motorBerhentiSemua();
  delay(500);
  jogMotor(false);
  motorBerhentiSemua();
  Serial.println(F("\n  Jog selesai, kedua motor berhenti."));
  Serial.println(F("  Definisi: SIGN_MOT = +1 bila IN1=HIGH membuat roda maju; SIGN_ENC = +1 bila hitungan naik saat roda maju."));
}

// =====================================================================
//  Menu 'e' - Pantau encoder (putar roda dengan tangan)
// =====================================================================
void menuEncoder() {
  garis();
  Serial.println(F("[T3] Pantau encoder 15 s (motor MATI). Putar roda dengan tangan. Tombol apa pun = berhenti."));
  Serial.println(F("  Untuk CPR: putar roda tepat 10 putaran, hitungan 1x / 10 = CPR_1X (asumsi 517)."));
  kurasSerial();
  resetEncoder();
  unsigned long t0 = millis();
  while (millis() - t0 < 15000 && !adaTombol()) {
    long ki, ka;
    bacaEncoder(ki, ka);
    Serial.printf("  KI 4x=%+7ld (1x %+6ld)   KA 4x=%+7ld (1x %+6ld)   C1/C2 KI=%d%d KA=%d%d\n",
                  ki, ki / 4, ka, ka / 4, digitalRead(ENC_KI_A), digitalRead(ENC_KI_B),
                  digitalRead(ENC_KA_A), digitalRead(ENC_KA_B));
    delay(250);
  }
  kurasSerial();
}

// =====================================================================
//  Menu 'a' - Jalankan semua uji pasif (tanpa aktuator berdaya besar)
// =====================================================================
void menuSemuaPasif() {
  menuInfoChip();
  menuVbat();
  menuUrutanTof();
  menuScanI2C();
  menuImu();
  menuRtcBaca();
  menuHcsr04();
  garis();
  int a = digitalRead(PIN_TCRT_KI), b = digitalRead(PIN_TCRT_KA);
  Serial.printf("[T6] TCRT KI=%d KA=%d (menu 'l' untuk pantau)\n", a, b);
  Serial.println(F("Uji aktuator (m motor, v suction, c LED, z buzzer, x PCF) dijalankan terpisah."));
}

// =====================================================================
void tampilkanMenu() {
  Serial.println();
  Serial.println(F("==================== BRINGUP PCB UTAMA ===================="));
  Serial.println(F("  i = info chip (flash, PSRAM)                     [T2]"));
  Serial.println(F("  b = tegangan baterai (VBAT)                      [T1]"));
  Serial.println(F("  s = pindai I2C + PASS/FAIL                       [T2]"));
  Serial.println(F("  t = urutan alamat VL53L0X + model ID             [T5]"));
  Serial.println(F("  x = uji tulis PCF8574                            [T5]"));
  Serial.println(F("  u = baca HC-SR04 US1/US4/US5 (cm)                [T5]"));
  Serial.println(F("  g = IMU MPU9250 WHO_AM_I + data mentah           [T6]"));
  Serial.println(F("  r = baca waktu DS3231     T = set RTC (waktu kompilasi)"));
  Serial.println(F("  l = pantau TCRT5000                              [T6]"));
  Serial.println(F("  c = siklus LED RGB        z = buzzer             [T8]"));
  Serial.println(F("  v = suction 3 s (konfirmasi Y)                   [T4]"));
  Serial.println(F("  m = jog motor + konstanta SIGN (konfirmasi Y)    [T3]"));
  Serial.println(F("  e = pantau encoder (putar roda dengan tangan)    [T3]"));
  Serial.println(F("  a = semua uji pasif berurutan"));
  Serial.println(F("  0 = semua keluaran ke keadaan aman   ? = menu"));
  Serial.println(F("==========================================================="));
}

void setup() {
  keadaanAman();   // PALING AWAL: motor berhenti, suction LOW, XSHUT LOW, LED/buzzer mati

  // PWM motor 20 kHz (API berbeda antara core 2.x dan 3.x)
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(PIN_PWMA, PWM_FREQ_HZ);
  analogWriteFrequency(PIN_PWMB, PWM_FREQ_HZ);
#else
  analogWriteFrequency(PWM_FREQ_HZ);
#endif
  analogWrite(PIN_PWMA, 0);
  analogWrite(PIN_PWMB, 0);

  Serial.begin(115200);
  delay(500);

  // Masukan
  pinMode(PIN_ECHO_US1, INPUT); pinMode(PIN_ECHO_US4, INPUT); pinMode(PIN_ECHO_US5, INPUT);  // pembagi 2k ke GND
  pinMode(PIN_TCRT_KI, INPUT);  pinMode(PIN_TCRT_KA, INPUT);
  pinMode(ENC_KI_A, INPUT_PULLUP); pinMode(ENC_KI_B, INPUT_PULLUP);
  pinMode(ENC_KA_A, INPUT_PULLUP); pinMode(ENC_KA_B, INPUT_PULLUP);
  stKi = (digitalRead(ENC_KI_A) << 1) | digitalRead(ENC_KI_B);
  stKa = (digitalRead(ENC_KA_A) << 1) | digitalRead(ENC_KA_B);
  attachInterrupt(digitalPinToInterrupt(ENC_KI_A), isrKi, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KI_B), isrKi, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KA_A), isrKa, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KA_B), isrKa, CHANGE);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);

  Wire.begin((int)PIN_SDA, (int)PIN_SCL, (uint32_t)I2C_HZ);   // cast: hindari ambigu dengan begin(slaveAddr, ...) di core 2.x
  pcfAda = i2cAda(ADDR_PCF);
  if (pcfAda) pcfTulis(0x00);   // PCF8574 menyala dengan semua pin HIGH -> TRIG LOW-kan

  Serial.println();
  Serial.println(F("Bringup PCB Utama siap. Semua keluaran dalam keadaan aman:"));
  Serial.println(F("  motor berhenti, suction LOW, XSHUT ToF LOW (kedua VL53L0X mati), TRIG HC-SR04 LOW."));
  Serial.println(pcfAda ? F("  PCF8574 terdeteksi di 0x20.") : F("  PCF8574 belum terdeteksi (normal bila U3 belum dipasang)."));
  tampilkanMenu();
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\r' || c == '\n' || c == ' ') return;
  kurasSerial();
  switch (c) {
    case 'i': menuInfoChip(); break;
    case 'b': menuVbat(); break;
    case 's': menuScanI2C(); break;
    case 't': menuUrutanTof(); break;
    case 'x': menuPcf(); break;
    case 'u': menuHcsr04(); break;
    case 'g': menuImu(); break;
    case 'r': menuRtcBaca(); break;
    case 'T': menuRtcSet(); break;
    case 'l': menuTcrt(); break;
    case 'c': menuLed(); break;
    case 'z': menuBuzzer(); break;
    case 'v': menuSuction(); break;
    case 'm': menuMotor(); break;
    case 'e': menuEncoder(); break;
    case 'a': menuSemuaPasif(); break;
    case '0':
      motorBerhentiSemua();
      digitalWrite(PIN_SUCTION, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      setLed(0, 0, 0);
      if (i2cAda(ADDR_PCF)) pcfTulis(0x00);
      Serial.println(F("-> keadaan aman (XSHUT tidak diubah)."));
      break;
    case '?': tampilkanMenu(); break;
    default: Serial.printf("Perintah '%c' tidak dikenal. Ketik ? untuk menu.\n", c); break;
  }
}
