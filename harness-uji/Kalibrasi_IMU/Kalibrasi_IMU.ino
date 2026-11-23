// ============================================================
//  FIRMWARE UJI - KALIBRASI IMU GY-9250 (MPU9250 + AK8963)
//  Fungsi : pindai I2C, cek WHO_AM_I, kalibrasi bias gyro/accel,
//           kalibrasi magnetometer (hard/soft iron), simpan ke NVS
//  Catatan: hasil kalibrasi dipakai firmware final Modul D
//           (perintah 'd' mencetak konstanta siap salin).
//  Board  : ESP32-S3 (Arduino-ESP32 core 3.x), Serial 115200
// ============================================================
//  Perintah Serial Monitor (ketik 1 huruf lalu Enter):
//    w = pindai I2C + cek WHO_AM_I
//    g = kalibrasi gyro + accel (modul DIAM dan DATAR)
//    m = kalibrasi magnetometer 3D (putar ke segala arah 30 s)
//    r = kalibrasi magnetometer 2D (robot diputar di lantai 30 s)
//    p = tampil data terkoreksi (tekan lagi untuk berhenti)
//    s = simpan ke NVS     l = muat dari NVS     c = hapus kalibrasi
//    d = cetak konstanta kalibrasi untuk firmware
// ============================================================

#include <Wire.h>
#include <Preferences.h>

// ---------- Peta pin (sama dengan PCB utama rev 004) ----------
#define PIN_SDA 47
#define PIN_SCL 48
#define MPU_ADDR 0x69   // AD0 ke 3V3 (0x68 bila AD0 ke GND / tidak disambung)
#define MAG_ADDR 0x0C   // AK8963, terlihat lewat mode bypass MPU9250

Preferences pref;
float gb[3] = {0, 0, 0};      // bias gyro [dps]
float ab[3] = {0, 0, 0};      // bias accel [g]
float mo[3] = {0, 0, 0};      // offset magnetometer (hard iron) [uT]
float ms[3] = {1, 1, 1};      // skala magnetometer (soft iron, pendekatan diagonal)
float asa[3] = {1, 1, 1};     // koreksi sensitivitas pabrik AK8963
bool adaMag = false;
bool cetak = false;
unsigned long tCetak = 0;

// ---------- I2C dasar ----------
void tulis(uint8_t dev, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool baca(uint8_t dev, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)dev, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

uint8_t baca1(uint8_t dev, uint8_t reg) {
  uint8_t v = 0xFF;
  baca(dev, reg, &v, 1);
  return v;
}

// ---------- Inisialisasi sensor ----------
void mulaiIMU() {
  tulis(MPU_ADDR, 0x6B, 0x80); delay(100);   // reset
  tulis(MPU_ADDR, 0x6B, 0x01); delay(50);    // clock PLL gyro
  tulis(MPU_ADDR, 0x1A, 0x03);               // DLPF gyro 41 Hz
  tulis(MPU_ADDR, 0x19, 0x04);               // sample rate 200 Hz
  tulis(MPU_ADDR, 0x1B, 0x00);               // gyro +-250 dps (131 LSB/dps)
  tulis(MPU_ADDR, 0x1C, 0x00);               // accel +-2 g (16384 LSB/g)
  tulis(MPU_ADDR, 0x1D, 0x03);               // DLPF accel 41 Hz
  tulis(MPU_ADDR, 0x6A, 0x00);               // I2C master internal mati
  tulis(MPU_ADDR, 0x37, 0x02);               // bypass: AK8963 muncul di bus (0x0C)
  delay(10);

  adaMag = (baca1(MAG_ADDR, 0x00) == 0x48);  // WIA AK8963
  if (adaMag) {
    tulis(MAG_ADDR, 0x0A, 0x00); delay(10);  // power down
    tulis(MAG_ADDR, 0x0A, 0x0F); delay(10);  // mode fuse ROM
    uint8_t r[3];
    if (baca(MAG_ADDR, 0x10, r, 3))
      for (int i = 0; i < 3; i++) asa[i] = (r[i] - 128) / 256.0f + 1.0f;
    tulis(MAG_ADDR, 0x0A, 0x00); delay(10);
    tulis(MAG_ADDR, 0x0A, 0x16); delay(10);  // 16 bit, kontinu 100 Hz
  }
}

// ---------- Baca data mentah ----------
bool bacaAG(float a[3], float g[3]) {
  uint8_t b[14];
  if (!baca(MPU_ADDR, 0x3B, b, 14)) return false;
  for (int i = 0; i < 3; i++) {
    a[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / 16384.0f;
    g[i] = (int16_t)((b[8 + 2 * i] << 8) | b[9 + 2 * i]) / 131.0f;
  }
  return true;
}

// hasil dalam uT, sudah diputar ke sumbu accel/gyro (X_mag = Y_acc, Y_mag = X_acc, Z_mag = -Z_acc)
bool bacaM(float m[3]) {
  if (!adaMag) return false;
  if (!(baca1(MAG_ADDR, 0x02) & 0x01)) return false;   // ST1 DRDY
  uint8_t b[7];
  if (!baca(MAG_ADDR, 0x03, b, 7)) return false;       // HXL..HZH + ST2
  if (b[6] & 0x08) return false;                       // overflow
  float r[3];
  for (int i = 0; i < 3; i++) r[i] = (int16_t)((b[2 * i + 1] << 8) | b[2 * i]) * 0.15f * asa[i];
  m[0] = r[1];
  m[1] = r[0];
  m[2] = -r[2];
  return true;
}

// ---------- Perintah ----------
void pindaiI2C() {
  Serial.print("Perangkat I2C ditemukan:");
  int n = 0;
  for (uint8_t ad = 1; ad < 127; ad++) {
    Wire.beginTransmission(ad);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", ad); n++; }
  }
  Serial.println(n ? "" : " TIDAK ADA - cek kabel VCC/GND/SDA/SCL");

  uint8_t who = baca1(MPU_ADDR, 0x75);
  Serial.printf("WHO_AM_I (0x%02X) = 0x%02X -> ", MPU_ADDR, who);
  switch (who) {
    case 0x71: Serial.println("MPU9250 (asli, ada magnetometer)"); break;
    case 0x73: Serial.println("MPU9255 (setara MPU9250, ada magnetometer)"); break;
    case 0x70: Serial.println("MPU6500 - TIDAK ADA magnetometer (modul GY-9250 palsu/varian 6500)"); break;
    case 0x68: Serial.println("MPU6050 (GY-521) - 6 sumbu, TIDAK ADA magnetometer"); break;
    case 0xFF: Serial.println("tidak terbaca - cek alamat (AD0 ke 3V3 = 0x69) dan kabel"); break;
    default:   Serial.println("tidak dikenal"); break;
  }
  Serial.printf("Magnetometer AK8963 (0x0C): %s\n", adaMag ? "ADA (WIA = 0x48)" : "TIDAK ADA");
}

void kalibGyroAccel() {
  Serial.println("Letakkan DIAM dan DATAR (komponen modul menghadap ATAS). Mulai 3 detik lagi...");
  delay(3000);
  double sa[3] = {0, 0, 0}, sg[3] = {0, 0, 0}, sg2[3] = {0, 0, 0};
  float a[3], g[3];
  int n = 0;
  while (n < 2000) {
    if (bacaAG(a, g)) {
      for (int i = 0; i < 3; i++) { sa[i] += a[i]; sg[i] += g[i]; sg2[i] += g[i] * g[i]; }
      n++;
    }
    delay(2);
  }
  float sd = 0;
  for (int i = 0; i < 3; i++) {
    gb[i] = sg[i] / n;
    ab[i] = sa[i] / n;
    sd = fmaxf(sd, sqrtf(fmaxf(0, sg2[i] / n - gb[i] * gb[i])));
  }
  ab[2] -= 1.0f;   // sumbu Z seharusnya +1 g saat datar
  Serial.printf("Bias gyro [dps]: %.3f %.3f %.3f | bias accel [g]: %.4f %.4f %.4f\n",
                gb[0], gb[1], gb[2], ab[0], ab[1], ab[2]);
  if (sd > 0.5f) Serial.println("PERINGATAN: modul bergerak saat kalibrasi (noise gyro besar). Ulangi 'g'.");
  if (fabsf(ab[0]) > 0.15f || fabsf(ab[1]) > 0.15f) Serial.println("PERINGATAN: modul tidak datar. Ulangi 'g' di permukaan datar.");
  Serial.println("Selesai. Ketik 's' untuk menyimpan.");
}

void kalibMag(bool mode2D) {
  if (!adaMag) { Serial.println("Magnetometer tidak ada (kemungkinan MPU6500). Kalibrasi mag dilewati."); return; }
  if (mode2D) Serial.println("Mode 2D: putar ROBOT pelan di lantai 2-3 putaran penuh selama 30 detik. Mulai 3 detik lagi...");
  else Serial.println("Mode 3D: putar modul pelan ke SEGALA arah (gerakan angka 8, balik atas-bawah) 30 detik. Mulai 3 detik lagi...");
  delay(3000);
  float mn[3] = {1e9, 1e9, 1e9}, mx[3] = {-1e9, -1e9, -1e9}, m[3];
  unsigned long t0 = millis(), tl = 0;
  int n = 0;
  while (millis() - t0 < 30000) {
    if (bacaM(m)) {
      for (int i = 0; i < 3; i++) { mn[i] = fminf(mn[i], m[i]); mx[i] = fmaxf(mx[i], m[i]); }
      n++;
    }
    if (millis() - tl > 1000) { tl = millis(); Serial.printf("  sisa %lu s\n", (30000 - (millis() - t0)) / 1000); }
    delay(10);
  }
  if (n < 100) { Serial.println("Data magnetometer terlalu sedikit. Ulangi."); return; }
  int k = mode2D ? 2 : 3;     // mode 2D hanya X dan Y yang dikalibrasi
  float d[3], rata = 0;
  for (int i = 0; i < k; i++) { mo[i] = (mx[i] + mn[i]) / 2; d[i] = (mx[i] - mn[i]) / 2; rata += d[i] / k; }
  for (int i = 0; i < k; i++) ms[i] = d[i] > 1 ? rata / d[i] : 1;
  Serial.printf("Offset mag [uT]: %.2f %.2f %.2f | skala: %.3f %.3f %.3f\n", mo[0], mo[1], mo[2], ms[0], ms[1], ms[2]);
  for (int i = 0; i < k; i++)
    if (ms[i] < 0.7f || ms[i] > 1.3f) Serial.println("PERINGATAN: rentang satu sumbu jauh berbeda - putaran kurang lengkap atau ada logam/magnet dekat. Ulangi.");
  Serial.println("Selesai. Ketik 's' untuk menyimpan.");
}

void tampil() {
  float a[3], g[3], m[3];
  if (!bacaAG(a, g)) { Serial.println("gagal baca MPU"); return; }
  for (int i = 0; i < 3; i++) { a[i] -= ab[i]; g[i] -= gb[i]; }
  float roll = atan2f(a[1], a[2]) * 57.2958f;
  float pitch = atan2f(-a[0], sqrtf(a[1] * a[1] + a[2] * a[2])) * 57.2958f;
  Serial.printf("acc[g] %6.3f %6.3f %6.3f | gyro[dps] %6.2f %6.2f %6.2f | roll %6.1f pitch %6.1f",
                a[0], a[1], a[2], g[0], g[1], g[2], roll, pitch);
  if (bacaM(m)) {
    for (int i = 0; i < 3; i++) m[i] = (m[i] - mo[i]) * ms[i];
    float hdg = atan2f(m[1], m[0]) * 57.2958f;   // sumbu robot: +X depan, +Y kiri, +Z atas
    if (hdg < 0) hdg += 360;
    Serial.printf(" | mag[uT] %6.1f %6.1f %6.1f | |B| %5.1f | arah %5.1f", m[0], m[1], m[2],
                  sqrtf(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]), hdg);
  }
  Serial.println();
}

void simpan() {
  pref.begin("imu", false);
  pref.putBytes("gb", gb, sizeof(gb)); pref.putBytes("ab", ab, sizeof(ab));
  pref.putBytes("mo", mo, sizeof(mo)); pref.putBytes("ms", ms, sizeof(ms));
  pref.end();
  Serial.println("Tersimpan ke NVS (namespace \"imu\").");
}

void muat() {
  pref.begin("imu", true);
  if (pref.isKey("gb")) {
    pref.getBytes("gb", gb, sizeof(gb)); pref.getBytes("ab", ab, sizeof(ab));
    pref.getBytes("mo", mo, sizeof(mo)); pref.getBytes("ms", ms, sizeof(ms));
    Serial.println("Kalibrasi dimuat dari NVS.");
  } else {
    Serial.println("Belum ada kalibrasi tersimpan.");
  }
  pref.end();
}

void hapus() {
  for (int i = 0; i < 3; i++) { gb[i] = ab[i] = mo[i] = 0; ms[i] = 1; }
  pref.begin("imu", false); pref.clear(); pref.end();
  Serial.println("Kalibrasi dihapus.");
}

void cetakKonstanta() {
  Serial.println("// ---- salin ke firmware Modul D ----");
  Serial.printf("const float GYRO_BIAS[3]  = {%.4ff, %.4ff, %.4ff};  // dps\n", gb[0], gb[1], gb[2]);
  Serial.printf("const float ACC_BIAS[3]   = {%.4ff, %.4ff, %.4ff};  // g\n", ab[0], ab[1], ab[2]);
  Serial.printf("const float MAG_OFFSET[3] = {%.3ff, %.3ff, %.3ff};  // uT (sumbu accel)\n", mo[0], mo[1], mo[2]);
  Serial.printf("const float MAG_SCALE[3]  = {%.4ff, %.4ff, %.4ff};\n", ms[0], ms[1], ms[2]);
}

void menu() {
  Serial.println("\nPerintah: w=pindai/WHO_AM_I  g=kalib gyro+accel  m=kalib mag 3D  r=kalib mag 2D (robot)");
  Serial.println("          p=tampil data  s=simpan  l=muat  c=hapus  d=cetak konstanta");
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  mulaiIMU();
  Serial.println("\n=== Kalibrasi IMU GY-9250 ===");
  pindaiI2C();
  muat();
  menu();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'w': mulaiIMU(); pindaiI2C(); break;
      case 'g': cetak = false; kalibGyroAccel(); break;
      case 'm': cetak = false; kalibMag(false); break;
      case 'r': cetak = false; kalibMag(true); break;
      case 'p': cetak = !cetak; break;
      case 's': simpan(); break;
      case 'l': muat(); break;
      case 'c': hapus(); break;
      case 'd': cetakKonstanta(); break;
      case '\n': case '\r': case ' ': break;
      default: menu(); break;
    }
  }
  if (cetak && millis() - tCetak > 200) {
    tCetak = millis();
    tampil();
  }
}
