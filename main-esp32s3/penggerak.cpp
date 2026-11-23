// penggerak.cpp - inner loop kecepatan roda (20 Hz) + odometri
// Asal: Uji_PID_Kecepatan_Roda_v2 (PID per-sampel, derivatif dari pengukuran, anti-windup
// bersyarat, profil trapesium, odometri RK2, pengaman stall/tanda salah dengan fault terkunci).
// Tambahan: batas gerak dari supervisor keselamatan, rem darurat, fusi heading gyro.
#include "penggerak.h"
#include "config.h"
#include "param.h"
#include "sensor_imu.h"

// ---------------- Encoder (ISR) ----------------
static volatile long sCntKi = 0, sCntKa = 0;    // hitungan 4x mentah
static volatile uint8_t sStKi = 0, sStKa = 0;
static const DRAM_ATTR int8_t QDEC[16] = {0, -1, +1, 0, +1, 0, 0, -1, -1, 0, 0, +1, 0, +1, -1, 0};

static void IRAM_ATTR isrEncKi() {
  uint8_t s = (digitalRead(PIN_ENC_KI_A) << 1) | digitalRead(PIN_ENC_KI_B);
  sCntKi += QDEC[(sStKi << 2) | s];
  sStKi = s;
}
static void IRAM_ATTR isrEncKa() {
  uint8_t s = (digitalRead(PIN_ENC_KA_A) << 1) | digitalRead(PIN_ENC_KA_B);
  sCntKa += QDEC[(sStKa << 2) | s];
  sStKa = s;
}

// ---------------- State roda ----------------
struct Roda {
  float ref = 0, refRamp = 0, v = 0, vPrev = 0, integ = 0, u = 0;
};
struct Pengaman {
  bool aktif = false;
  uint32_t tMulai = 0;
  float eAwal = 0;
};

static Roda sKi, sKa;
static Pengaman sGKi, sGKa;
static long sLastKi = 0, sLastKa = 0;
static bool sFault = false;
static char sAlasan[160] = "";
static BatasGerak sBatas = BATAS_TIDAK;
static bool sRemDarurat = false;
static Pose sPose = {0, 0, 0, 0};
static float sVTerukur = 0, sWTerukur = 0;
static float sVCmd = 0, sWCmd = 0;
// Mode PWM mentah (uji)
static bool sMentah = false;
static int sPwmKi = 0, sPwmKa = 0;
static uint32_t sMentahSampai = 0;

static void tulisMotor(uint8_t in1, uint8_t in2, uint8_t pwmPin, float u, int sign) {
  u *= sign;
  int pwm = (int)constrain(fabsf(u), 0.0f, U_MAX);
  if (pwm > 0 && pwm < U_MIN_GERAK) pwm = (int)U_MIN_GERAK;
  if (u > 0)      { digitalWrite(in1, HIGH); digitalWrite(in2, LOW);  }
  else if (u < 0) { digitalWrite(in1, LOW);  digitalWrite(in2, HIGH); }
  else            { digitalWrite(in1, LOW);  digitalWrite(in2, LOW);  }   // coast
  analogWrite(pwmPin, pwm);
}

static void motorNol() {
  tulisMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0, 1);
  tulisMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0, 1);
}

static void nolkanRoda(Roda &w) { w.ref = w.refRamp = w.u = w.integ = 0; }

static float rampTarget(float now, float target, float aMaks) {
  float dmax = aMaks * TS * TICK_PER_MPS;
  if (target > now + dmax) return now + dmax;
  if (target < now - dmax) return now - dmax;
  return target;
}

static void pid(Roda &w) {
  float e = w.refRamp - w.v;
  float d = -(w.v - w.vPrev);            // derivatif dari pengukuran
  float uTanpaI = P.kp * e + P.kd * d;
  float iBaru = w.integ + e;
  float u = uTanpaI + P.ki * iBaru;
  if (fabsf(u) <= U_MAX || (u > U_MAX && e < 0) || (u < -U_MAX && e > 0)) w.integ = iBaru;
  u = uTanpaI + P.ki * w.integ;
  w.u = constrain(u, -U_MAX, U_MAX);
  if (w.refRamp == 0 && w.ref == 0 && fabsf(w.v) < 0.5f) { w.u = 0; w.integ = 0; }   // berhenti bersih
  w.vPrev = w.v;
}

static void kunciFault(const char *alasan) {
  sFault = true;
  strncpy(sAlasan, alasan, sizeof(sAlasan) - 1);
  sAlasan[sizeof(sAlasan) - 1] = 0;
  nolkanRoda(sKi); nolkanRoda(sKa);
  sVCmd = sWCmd = 0;
  sMentah = false;
  motorNol();
  Serial.printf("# FAULT PENGGERAK: %s\n", sAlasan);
}

// Pengaman per roda (sama dengan Uji_PID v2). true bila fault.
static bool cekPengaman(Roda &w, Pengaman &g, const char *nama, const char *suf, uint32_t now) {
  float e = w.refRamp - w.v;
  bool jenuh = fabsf(w.u) >= PWM_JENUH;
  bool macet = fabsf(w.v) < V_DIAM_TICK;
  bool lawanTanda = (w.refRamp * w.v < 0) && !macet;
  if (!jenuh || !(macet || lawanTanda)) { g.aktif = false; return false; }
  if (!g.aktif) { g.aktif = true; g.tMulai = now; g.eAwal = fabsf(e); return false; }
  if (now - g.tMulai <= T_JENUH_MS) return false;
  char buf[160];
  if (macet) {
    snprintf(buf, sizeof(buf), "roda %s MACET: |PWM|=%.0f selama > %lu ms, kecepatan %.2f tick/50ms. "
             "Cek roda tertahan / kabel motor / encoder.", nama, fabsf(w.u), (unsigned long)T_JENUH_MS, w.v);
    kunciFault(buf);
    return true;
  }
  if (fabsf(e) >= g.eAwal) {
    snprintf(buf, sizeof(buf), "roda %s LARI / TANDA SALAH: kecepatan %.2f lawan target %.2f. "
             "Cek SIGN_MOT_%s / SIGN_ENC_%s di config.h.", nama, w.v, w.refRamp, suf, suf);
    kunciFault(buf);
    return true;
  }
  return false;
}

static void terapkanKinematika(float v, float w) {
  // kinematika invers differential drive, jaga rasio v/w bila jenuh
  float vR = v + w * TRACK_L / 2.0f;
  float vL = v - w * TRACK_L / 2.0f;
  float m = fmaxf(fabsf(vR), fabsf(vL));
  if (m > V_MAKS) { vR *= V_MAKS / m; vL *= V_MAKS / m; }
  sKi.ref = vL * TICK_PER_MPS;
  sKa.ref = vR * TICK_PER_MPS;
}

// =====================================================================
void penggerakMulai() {
  const uint8_t keluaran[] = {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB};
  for (uint8_t p : keluaran) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  analogWriteFrequency(PIN_PWMA, PWM_FREQ_HZ);
  analogWriteFrequency(PIN_PWMB, PWM_FREQ_HZ);
#else
  analogWriteFrequency(PWM_FREQ_HZ);
#endif
  motorNol();

  // Encoder JGA25-370 dicatu 3V3 -> pull-up internal
  pinMode(PIN_ENC_KI_A, INPUT_PULLUP); pinMode(PIN_ENC_KI_B, INPUT_PULLUP);
  pinMode(PIN_ENC_KA_A, INPUT_PULLUP); pinMode(PIN_ENC_KA_B, INPUT_PULLUP);
  sStKi = (digitalRead(PIN_ENC_KI_A) << 1) | digitalRead(PIN_ENC_KI_B);
  sStKa = (digitalRead(PIN_ENC_KA_A) << 1) | digitalRead(PIN_ENC_KA_B);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_KI_A), isrEncKi, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_KI_B), isrEncKi, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_KA_A), isrEncKa, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_KA_B), isrEncKa, CHANGE);
  penggerakResetOdo();
}

void penggerakSetKecepatan(float vMps, float wRadps) {
  if (sFault || sMentah) return;
  if (sBatas == BATAS_REM || sBatas == BATAS_MATI) return;
  sRemDarurat = false;
  sVCmd = vMps; sWCmd = wRadps;
}

void penggerakBerhenti(bool darurat) {
  sVCmd = sWCmd = 0;
  sMentah = false;
  if (darurat) sRemDarurat = true;
}

void penggerakSetBatas(BatasGerak b) {
  if (b == BATAS_REM && sBatas != BATAS_REM) penggerakBerhenti(true);
  if (b == BATAS_MATI && sBatas != BATAS_MATI) {
    sVCmd = sWCmd = 0;
    sMentah = false;
    nolkanRoda(sKi); nolkanRoda(sKa);
    motorNol();
  }
  sBatas = b;
}
BatasGerak penggerakBatas() { return sBatas; }

void penggerakPwmMentah(int pwmKi, int pwmKa, uint32_t durasiMs) {
  if (sFault || sBatas != BATAS_TIDAK) return;
  sMentah = true;
  sPwmKi = constrain(pwmKi, -255, 255);
  sPwmKa = constrain(pwmKa, -255, 255);
  sMentahSampai = millis() + durasiMs;
  nolkanRoda(sKi); nolkanRoda(sKa);
}
bool penggerakModeMentah() { return sMentah; }

bool penggerakFault() { return sFault; }
const char *penggerakAlasanFault() { return sAlasan; }
void penggerakLepasFault() {
  sFault = false;
  sAlasan[0] = 0;
  sGKi.aktif = sGKa.aktif = false;
  nolkanRoda(sKi); nolkanRoda(sKa);
  sVCmd = sWCmd = 0;
}

Pose penggerakPose() { return sPose; }

void penggerakResetOdo() {
  noInterrupts(); long a = sCntKi, b = sCntKa; interrupts();
  sLastKi = a; sLastKa = b;
  sPose = {0, 0, 0, 0};
  imuAmbilDeltaSudut();   // buang akumulasi gyro lama
}

float penggerakV() { return sVTerukur; }
float penggerakW() { return sWTerukur; }

bool penggerakDiam() {
  return fabsf(sKi.v) < 0.5f && fabsf(sKa.v) < 0.5f && sKi.refRamp == 0 && sKa.refRamp == 0 && !sMentah;
}

void penggerakTick(long &kiri, long &kanan) {
  noInterrupts(); long a = sCntKi, b = sCntKa; interrupts();
  kiri = SIGN_ENC_KI * a / 4;
  kanan = SIGN_ENC_KA * b / 4;
}

void penggerakUpdate(uint32_t now) {
  // 1) ukur kecepatan (tick 1x per 50 ms, bertanda)
  noInterrupts(); long cKi = sCntKi, cKa = sCntKa; interrupts();
  float dKi = SIGN_ENC_KI * (cKi - sLastKi) / 4.0f;
  float dKa = SIGN_ENC_KA * (cKa - sLastKa) / 4.0f;
  sLastKi = cKi; sLastKa = cKa;
  sKi.v = dKi; sKa.v = dKa;

  // 2) odometri RK2, heading difusikan dengan gyro (complementary)
  float sL = dKi * M_PER_TICK, sR = dKa * M_PER_TICK;
  float ds = (sR + sL) / 2.0f;
  float dthEnc = (sR - sL) / TRACK_L;
  float dthGyro = imuAmbilDeltaSudut();
  float dth = imuOk() ? (P.alphaGyro * dthGyro + (1.0f - P.alphaGyro) * dthEnc) : dthEnc;
  sPose.x += ds * cosf(sPose.th + dth / 2.0f);
  sPose.y += ds * sinf(sPose.th + dth / 2.0f);
  sPose.th += dth;
  sPose.s += ds;
  sVTerukur = ds / TS;
  sWTerukur = dth / TS;

  // 3) mode PWM mentah (uji, tanpa PID) - tetap tunduk pada batas keselamatan
  if (sMentah) {
    if ((int32_t)(now - sMentahSampai) >= 0 || sBatas != BATAS_TIDAK || sFault) {
      sMentah = false;
      motorNol();
      Serial.println(F("# uji PWM mentah selesai, motor berhenti."));
    } else {
      tulisMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, sPwmKi, SIGN_MOT_KI);
      tulisMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, sPwmKa, SIGN_MOT_KA);
    }
    return;
  }

  // 4) batas gerak dari supervisor keselamatan
  if (sBatas == BATAS_MATI || sFault) {
    nolkanRoda(sKi); nolkanRoda(sKa);
    motorNol();
    return;
  }
  float v = sVCmd, w = sWCmd;
  if (sBatas == BATAS_REM) { v = 0; w = 0; sRemDarurat = true; }
  if (sBatas == BATAS_LARANG_MAJU) {
    if (v >= 0) { v = 0; w = 0; sRemDarurat = true; }
    else w = 0;   // hanya mundur lurus
  }
  terapkanKinematika(v, w);

  // 5) profil akselerasi + PID
  float a = sRemDarurat ? P.aRem : P.aMaks;
  sKi.refRamp = rampTarget(sKi.refRamp, sKi.ref, a);
  sKa.refRamp = rampTarget(sKa.refRamp, sKa.ref, a);
  pid(sKi); pid(sKa);
  if (!cekPengaman(sKi, sGKi, "KIRI", "KI", now)) cekPengaman(sKa, sGKa, "KANAN", "KA", now);
  if (sFault) { motorNol(); return; }
  if (sRemDarurat && sKi.refRamp == 0 && sKa.refRamp == 0 && sVCmd == 0 && sWCmd == 0) sRemDarurat = false;

  tulisMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, sKi.u, SIGN_MOT_KI);
  tulisMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, sKa.u, SIGN_MOT_KA);
}

void penggerakCetak(Stream &s) {
  long a, b;
  penggerakTick(a, b);
  s.printf("# roda: ref %.1f/%.1f ramp %.1f/%.1f v %.1f/%.1f tick/50ms, u %.0f/%.0f, tick1x %ld/%ld, batas %d%s\n",
           sKi.ref, sKa.ref, sKi.refRamp, sKa.refRamp, sKi.v, sKa.v, sKi.u, sKa.u, a, b, (int)sBatas,
           sFault ? " FAULT" : "");
  s.printf("# pose: x %.3f m y %.3f m th %.1f deg s %.3f m | v %.3f m/s w %.2f rad/s\n",
           sPose.x, sPose.y, sPose.th * 180.0f / PI, sPose.s, sVTerukur, sWTerukur);
}
