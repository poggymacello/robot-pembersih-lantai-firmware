// sensor_imu.cpp - MPU9250 lewat register (pola Kalibrasi_IMU / Bringup_PCB)
#include "sensor_imu.h"
#include "config.h"
#include "i2c_bus.h"
#include <Preferences.h>

static bool sOk = false;
static uint8_t sWho = 0;
static bool sNvs = false;
static float sGb[3] = {0, 0, 0};    // bias gyro [dps] (NVS "gb")
static float sAb[3] = {0, 0, 0};    // bias accel [g]  (NVS "ab")
static float sGz = 0;               // rad/s terkoreksi
static float sAx = 0, sAy = 0, sAz = 1;   // g, tersaring
static float sTilt = 0;
static float sDeltaSudut = 0;       // rad akumulasi
static uint32_t sTPrevUs = 0;
static uint8_t sGagalBeruntun = 0;
// estimasi bias
static uint16_t sKalibTarget = 0, sKalibN = 0;
static double sKalibSum = 0, sKalibSum2 = 0;
static bool sKalibBerhasil = false;

static const float LSB_DPS = 65.5f;      // +-500 dps
static const float LSB_G = 16384.0f;     // +-2 g

static void muatNvs() {
  Preferences pref;
  sNvs = false;
  if (!pref.begin("imu", true)) return;
  if (pref.isKey("gb")) {
    pref.getBytes("gb", sGb, sizeof(sGb));
    if (pref.isKey("ab")) pref.getBytes("ab", sAb, sizeof(sAb));
    sNvs = true;
  }
  pref.end();
}

bool imuMulai() {
  int w = i2cBaca1(ADDR_MPU, 0x75);
  sWho = (w < 0) ? 0 : (uint8_t)w;
  // 0x71 MPU9250, 0x73 MPU9255, 0x70 MPU6500 (gyro/accel sama, tanpa magnetometer - cukup untuk firmware ini)
  if (!(sWho == 0x71 || sWho == 0x73 || sWho == 0x70)) { sWho = 0; sOk = false; muatNvs(); return false; }
  i2cTulisReg(ADDR_MPU, 0x6B, 0x80); delay(100);   // reset (hanya saat boot)
  i2cTulisReg(ADDR_MPU, 0x6B, 0x01); delay(20);    // clock PLL gyro
  i2cTulisReg(ADDR_MPU, 0x1A, 0x03);               // DLPF gyro 41 Hz
  i2cTulisReg(ADDR_MPU, 0x19, 0x04);               // 200 Hz
  i2cTulisReg(ADDR_MPU, 0x1B, 0x08);               // gyro +-500 dps (laju putar robot maks ~150 dps)
  i2cTulisReg(ADDR_MPU, 0x1C, 0x00);               // accel +-2 g
  i2cTulisReg(ADDR_MPU, 0x1D, 0x03);               // DLPF accel 41 Hz
  muatNvs();
  sTPrevUs = micros();
  sOk = true;
  return true;
}

bool imuOk() { return sOk; }
uint8_t imuWhoAmI() { return sWho; }
bool imuKalibDariNvs() { return sNvs; }
float imuGyroZ() { return sGz; }
float imuTiltDeg() { return sTilt; }
float imuBiasZdps() { return sGb[2]; }

float imuAmbilDeltaSudut() {
  float d = sDeltaSudut;
  sDeltaSudut = 0;
  return d;
}

void imuUpdate(uint32_t nowMs) {
  (void)nowMs;
  if (sWho == 0) return;   // tidak ada IMU sejak boot
  uint8_t b[14];
  uint32_t tUs = micros();
  if (!i2cBacaReg(ADDR_MPU, 0x3B, b, 14)) {
    if (++sGagalBeruntun >= 5) sOk = false;
    return;
  }
  sGagalBeruntun = 0;
  sOk = true;
  float ax = (int16_t)((b[0] << 8) | b[1]) / LSB_G - sAb[0];
  float ay = (int16_t)((b[2] << 8) | b[3]) / LSB_G - sAb[1];
  float az = (int16_t)((b[4] << 8) | b[5]) / LSB_G - sAb[2];
  float gzRaw = (int16_t)((b[12] << 8) | b[13]) / LSB_DPS;   // dps, sumbu chip

  // estimasi bias (robot diam)
  if (sKalibTarget) {
    sKalibSum += gzRaw; sKalibSum2 += (double)gzRaw * gzRaw; sKalibN++;
    if (sKalibN >= sKalibTarget) {
      double m = sKalibSum / sKalibN;
      double sd = sqrt(fmax(0.0, sKalibSum2 / sKalibN - m * m));
      sKalibBerhasil = sd < 0.5 && fabs(m) < 10.0;
      if (sKalibBerhasil) sGb[2] = (float)m;
      Serial.printf("# IMU: bias gyro z %s: %.3f dps (sd %.3f)\n", sKalibBerhasil ? "diperbarui" : "DITOLAK (robot bergerak?)",
                    (float)m, (float)sd);
      sKalibTarget = 0;
    }
  }

  float dt = (tUs - sTPrevUs) * 1e-6f;
  sTPrevUs = tUs;
  if (dt > 0.2f) dt = 0.2f;   // jeda panjang (mis. setelah blokir) jangan menimbulkan lompatan besar
  sGz = SIGN_GYRO_Z * (gzRaw - sGb[2]) * (PI / 180.0f);
  if (fabsf(sGz) < 0.002f) sGz = 0;   // zona mati derau kecil (~0,1 dps)
  sDeltaSudut += sGz * dt;

  // akselerometer tersaring untuk kemiringan
  const float a = 0.2f;
  sAx += a * (ax - sAx); sAy += a * (ay - sAy); sAz += a * (az - sAz);
  float n = sqrtf(sAx * sAx + sAy * sAy + sAz * sAz);
  if (n > 0.3f) {
    float c = constrain(sAz / n, -1.0f, 1.0f);
    sTilt = acosf(c) * 180.0f / PI;
  }
}

void imuMulaiKalibBias(uint16_t nSampel) {
  if (sWho == 0) { sKalibTarget = 0; sKalibBerhasil = false; return; }   // tidak ada IMU
  sKalibTarget = nSampel; sKalibN = 0; sKalibSum = sKalibSum2 = 0; sKalibBerhasil = false;
}
bool imuKalibBiasSibuk() { return sKalibTarget != 0 && sOk; }   // IMU gagal baca -> tidak ditunggu
bool imuKalibBiasBerhasil() { return sKalibBerhasil; }

void imuCetak(Stream &s) {
  s.printf("# IMU: WHO_AM_I 0x%02X %s, kalibrasi NVS %s, bias z %.3f dps, gz %.3f rad/s, tilt %.1f deg, acc %.2f %.2f %.2f g\n",
           sWho, sOk ? "OK" : "TIDAK OK", sNvs ? "ada" : "TIDAK ADA (jalankan Kalibrasi_IMU)", sGb[2], sGz, sTilt, sAx, sAy, sAz);
}
