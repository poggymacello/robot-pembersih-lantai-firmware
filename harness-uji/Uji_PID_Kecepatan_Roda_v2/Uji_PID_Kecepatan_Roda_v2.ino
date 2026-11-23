// =====================================================================
// Uji PID Kecepatan Roda v2 (ESP32-S3 DevKit N16R8 + TB6612FNG + 2x JGA25-370)
//
// Perubahan terhadap Uji_Feedback_Encoder.ino (hasil analisis monev B400 rev 002):
//  1. Pin mengikuti PCB utama (SPEK_BERSAMA): motor kiri 15/16/17, encoder kanan 6/7.
//  2. Encoder dekode 4x (CHANGE di C1 dan C2). Kecepatan tetap dinyatakan dalam
//     "tick 1x per 50 ms" supaya gain hasil simulasi bisa dipakai langsung.
//  3. Kecepatan bertanda (tanpa abs) + PWM dua arah -> bisa maju, mundur, putar di tempat.
//  4. Tanda encoder dan motor bisa dibalik per sisi (encoder kanan terbalik saat uji 15 Okt).
//  5. Gain awal 0,75 / 2,25 / 0,25 (bentuk per-sampel, Ts 50 ms), derivatif dari
//     pengukuran (tanpa derivative kick), anti-windup bersyarat.
//  6. Pembatas akselerasi target (profil trapesium) -> gerak halus, arus rendah,
//     perlambatan jauh di bawah batas tip-over (~2,8 m/s^2).
//  7. Odometri (x, y, theta) RK2 dan reset origin, untuk validasi jarak.
//  8. Mode uji step 1 / step 2 / ramp otomatis + log CSV untuk plot respons.
//
// Asumsi yang WAJIB dicek (lihat B400 rev 002, tabel parameter):
//  - CPR_1X = 517 tick per putaran roda pada dekode 1x (11 PPR x rasio 47). Ukur:
//    perintah "c" lalu putar roda tepat 10 putaran dengan tangan, baca jumlah tick.
//  - Diameter roda 67 mm, track 231 mm.
//
// PERINGATAN - KONSTANTA TANDA (SIGN_*) BELUM DIKONFIRMASI DI PCB:
//  Konektor motor kanan CN9 di PCB berurutan BO2-BO1 (terbalik terhadap breadboard) dan
//  encoder kanan menghitung negatif saat maju pada uji 15 Okt. Sebelum menjalankan sketch ini,
//  tentukan SIGN_MOT_KI/KA dan SIGN_ENC_KI/KA dengan Bringup_PCB (menu 'm', roda diangkat)
//  atau uji_modul_b, lalu isi di bawah. Tanda salah = umpan balik positif: PID mendorong
//  PWM ke penuh dan roda "lari". Pengaman (9.) menghentikan motor bila itu terjadi.
//  9. Pengaman: bila |PWM| >= 240 lebih dari 400 ms sementara roda macet (kecepatan ~0)
//     atau kecepatan berlawanan tanda dengan target dan error terus membesar, kedua motor
//     dihentikan dan fault dikunci (ketik "f" untuk melepas setelah penyebab diperbaiki).
// =====================================================================

// ---------------- Pin (PCB utama rev 003/004) ----------------
const int PIN_PWMA = 17, PIN_AIN2 = 16, PIN_AIN1 = 15;   // motor KIRI
const int PIN_PWMB = 9,  PIN_BIN1 = 18, PIN_BIN2 = 8;    // motor KANAN (STBY diikat 3V3 di PCB)
const int ENC_KI_A = 4, ENC_KI_B = 5;                    // encoder kiri C1, C2
const int ENC_KA_A = 6, ENC_KA_B = 7;                    // encoder kanan C1, C2

// ---------------- Arah (ubah kalau uji menunjukkan terbalik) ----------------
// Saat diperintah maju, tick kedua roda harus naik positif. Di uji 15 Okt tick kanan negatif.
const int SIGN_ENC_KI = +1, SIGN_ENC_KA = -1;
const int SIGN_MOT_KI = +1, SIGN_MOT_KA = +1;   // CN9 di PCB urutannya BO2-BO1

// ---------------- Parameter fisik ----------------
const float CPR_1X  = 517.0f;     // ASUMSI, wajib diukur
const float R_RODA  = 0.0335f;    // m (roda 67 mm)
const float TRACK_L = 0.231f;     // m (jarak pusat roda kiri-kanan)
const float TS      = 0.050f;     // s, periode loop kecepatan (gain di bawah untuk 50 ms)
const unsigned long TS_MS = 50;
const float M_PER_TICK = 2.0f * PI * R_RODA / CPR_1X;   // m per tick 1x
const float TICK_PER_MPS = TS / M_PER_TICK;              // (tick/50 ms) per (m/s) ~ 122,8

// ---------------- PID (bentuk per-sampel, satuan tick 1x per 50 ms) ----------------
float Kp = 0.75f, Ki = 2.25f, Kd = 0.25f;   // titik awal dari simulasi; setel ulang di lantai
const float U_MAX = 255.0f;                 // batas PWM
const float U_MIN_GERAK = 0.0f;             // kompensasi zona mati PWM (isi setelah diukur, mis. 20)

// ---------------- Profil gerak ----------------
float A_MAX = 0.5f;      // m/s^2, batas akselerasi target (simulasi trapesium 0,5: percepatan puncak ~0,8 m/s^2, arus ~0,24 A)
const float V_MAX = 0.30f;   // m/s, batas kecepatan B200

// ---------------- Pengaman (stall / tanda salah) ----------------
const float PWM_JENUH = 240.0f;          // |PWM| dianggap jenuh mulai nilai ini
const unsigned long T_JENUH_MS = 400;    // lama jenuh + kondisi buruk sebelum fault
const float V_DIAM = 1.0f;               // tick 1x per 50 ms (~8 mm/s) dianggap diam

struct Pengaman {
  bool aktif = false;          // jendela pengamatan sedang berjalan
  unsigned long tMulai = 0;
  float eAwal = 0;             // |error| saat jendela mulai
};
Pengaman gKi, gKa;
bool fault = false;
char alasanFault[160] = "";

// ---------------- State ----------------
volatile long cntKi = 0, cntKa = 0;          // hitungan 4x
volatile uint8_t stKi = 0, stKa = 0;
long lastKi = 0, lastKa = 0;

struct Roda {
  float ref = 0;        // target akhir (tick/50 ms)
  float refRamp = 0;    // target setelah pembatas akselerasi
  float v = 0;          // kecepatan terukur (tick/50 ms, bertanda)
  float vPrev = 0;
  float integ = 0;
  float u = 0;
};
Roda ki, ka;

float odoX = 0, odoY = 0, odoTh = 0;         // pose di frame dunia {W}, origin = posisi saat reset
unsigned long tPrev = 0, tStart = 0;
bool logCsv = true;

// Mode uji otomatis
enum Mode { MANUAL, UJI_STEP, UJI_RAMP };
Mode mode = MANUAL;
unsigned long tUji = 0;

// Tabel transisi kuadratur: indeks = (state lama << 2) | state baru
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

void tulisMotor(int pinIn1, int pinIn2, int pinPwm, float u, int sign) {
  u *= sign;
  int pwm = (int)constrain(fabsf(u), 0.0f, U_MAX);
  if (pwm > 0 && pwm < U_MIN_GERAK) pwm = (int)U_MIN_GERAK;
  if (u > 0)      { digitalWrite(pinIn1, HIGH); digitalWrite(pinIn2, LOW);  }
  else if (u < 0) { digitalWrite(pinIn1, LOW);  digitalWrite(pinIn2, HIGH); }
  else            { digitalWrite(pinIn1, LOW);  digitalWrite(pinIn2, LOW);  }  // coast
  analogWrite(pinPwm, pwm);
}

float rampTarget(float now, float target) {
  float dmax = A_MAX * TS * TICK_PER_MPS;   // perubahan maksimum per sampel (tick/50 ms)
  if (target > now + dmax) return now + dmax;
  if (target < now - dmax) return now - dmax;
  return target;
}

void pid(Roda &w) {
  float e = w.refRamp - w.v;
  // derivatif dari pengukuran (bukan dari error) -> tidak ada lonjakan saat target berubah
  float d = -(w.v - w.vPrev);
  float uTanpaI = Kp * e + Kd * d;
  float iBaru = w.integ + e;
  float u = uTanpaI + Ki * iBaru;
  // anti-windup bersyarat: integral hanya diperbarui bila output tidak jenuh,
  // atau bila error membantu keluar dari jenuh
  if (fabsf(u) <= U_MAX || (u > U_MAX && e < 0) || (u < -U_MAX && e > 0)) w.integ = iBaru;
  u = uTanpaI + Ki * w.integ;
  w.u = constrain(u, -U_MAX, U_MAX);
  if (w.refRamp == 0 && w.ref == 0 && fabsf(w.v) < 0.5f) { w.u = 0; w.integ = 0; }  // berhenti bersih
  w.vPrev = w.v;
}

void setKecepatan(float vMps, float wRadps) {
  // kinematika invers differential drive
  float vR = vMps + wRadps * TRACK_L / 2.0f;
  float vL = vMps - wRadps * TRACK_L / 2.0f;
  vR = constrain(vR, -V_MAX, V_MAX);
  vL = constrain(vL, -V_MAX, V_MAX);
  ki.ref = vL * TICK_PER_MPS;
  ka.ref = vR * TICK_PER_MPS;
}

void stopSemua() {
  setKecepatan(0, 0);
  mode = MANUAL;
}

void kunciFault(const char *alasan) {
  fault = true;
  strncpy(alasanFault, alasan, sizeof(alasanFault) - 1);
  alasanFault[sizeof(alasanFault) - 1] = 0;
  stopSemua();
  ki.refRamp = ka.refRamp = 0;
  ki.u = ka.u = 0;
  ki.integ = ka.integ = 0;
  tulisMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0, 1);
  tulisMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0, 1);
  Serial.printf("# FAULT: %s\n", alasanFault);
  Serial.println("# Kedua motor DIHENTIKAN. Perbaiki penyebab lalu ketik f untuk melepas fault.");
}

// Pengaman per roda, dipanggil tiap sampel setelah pid(). Kembalikan true bila fault.
bool cekPengaman(Roda &w, Pengaman &g, const char *nama, const char *suf, unsigned long now) {
  float e = w.refRamp - w.v;
  bool jenuh = fabsf(w.u) >= PWM_JENUH;
  bool macet = fabsf(w.v) < V_DIAM;
  bool lawanTanda = (w.refRamp * w.v < 0) && !macet;
  if (!jenuh || !(macet || lawanTanda)) { g.aktif = false; return false; }
  if (!g.aktif) { g.aktif = true; g.tMulai = now; g.eAwal = fabsf(e); return false; }
  if (now - g.tMulai <= T_JENUH_MS) return false;
  char buf[160];
  if (macet) {
    snprintf(buf, sizeof(buf),
             "roda %s MACET: |PWM|=%.0f >= %.0f selama > %lu ms tetapi kecepatan %.2f tick/50ms (~0). "
             "Cek roda tertahan / kabel motor / encoder.",
             nama, fabsf(w.u), PWM_JENUH, T_JENUH_MS, w.v);
    kunciFault(buf);
    return true;
  }
  if (fabsf(e) >= g.eAwal) {   // error tidak mengecil = terus membesar
    snprintf(buf, sizeof(buf),
             "roda %s LARI / TANDA SALAH: kecepatan %.2f berlawanan tanda dengan target %.2f, "
             "error %.1f -> %.1f. Cek SIGN_MOT_%s / SIGN_ENC_%s.",
             nama, w.v, w.refRamp, g.eAwal, fabsf(e), suf, suf);
    kunciFault(buf);
    return true;
  }
  return false;   // error sedang mengecil (roda sedang berbalik arah dengan benar)
}

void resetOdo() {
  noInterrupts(); cntKi = 0; cntKa = 0; interrupts();
  lastKi = lastKa = 0;
  odoX = odoY = odoTh = 0;
}

void bacaSerial() {
  if (!Serial.available()) return;
  String s = Serial.readStringUntil('\n');
  s.trim();
  if (s.length() == 0) return;
  char c = s.charAt(0);
  float a = 0, b = 0;
  sscanf(s.c_str() + 1, "%f %f", &a, &b);
  if (fault && (c == 'v' || c == 't' || c == 'S' || c == 'R')) {
    Serial.printf("# DITOLAK, fault terkunci: %s\n# Ketik f untuk melepas fault.\n", alasanFault);
    return;
  }
  switch (c) {
    case 'v': setKecepatan(a, b); mode = MANUAL; break;          // v <m/s> <rad/s>
    case 't': ki.ref = ka.ref = a; mode = MANUAL; break;         // t <tick/50 ms> (seperti uji lama)
    case 's': stopSemua(); break;
    case 'p': Kp = a; Serial.printf("# Kp=%.3f\n", Kp); break;
    case 'i': Ki = a; Serial.printf("# Ki=%.3f\n", Ki); break;
    case 'd': Kd = a; Serial.printf("# Kd=%.3f\n", Kd); break;
    case 'a': A_MAX = a; Serial.printf("# A_MAX=%.2f m/s^2\n", A_MAX); break;
    case 'r': resetOdo(); Serial.println("# origin direset ke (0, 0, 0)"); break;
    case 'c': Serial.printf("# tick 1x kiri=%.1f kanan=%.1f\n", SIGN_ENC_KI * cntKi / 4.0f, SIGN_ENC_KA * cntKa / 4.0f); break;
    case 'S': resetOdo(); mode = UJI_STEP; tUji = millis(); Serial.println("# uji step mulai"); break;
    case 'R': resetOdo(); mode = UJI_RAMP; tUji = millis(); Serial.println("# uji ramp mulai"); break;
    case 'l': logCsv = !logCsv; break;
    case 'f':
      if (fault) {
        fault = false;
        gKi.aktif = gKa.aktif = false;
        ki.integ = ka.integ = 0;
        Serial.println("# fault dilepas. Motor tetap berhenti sampai ada perintah baru.");
      } else {
        Serial.println("# tidak ada fault.");
      }
      break;
    default: Serial.println("# perintah: v <m/s> <rad/s> | t <tick> | s | p/i/d <gain> | a <m/s2> | r | c | S | R | l | f (lepas fault)");
  }
}

// Urutan uji step: 0 -> step 1 (0,15 m/s) -> step 2 (0,30 m/s) -> 0, tiap 3 s.
// Dengan A_MAX besar (perintah "a 100") hasilnya respons step murni.
void jalankanUji() {
  float t = (millis() - tUji) / 1000.0f;
  if (mode == UJI_STEP) {
    if (t < 1)       setKecepatan(0.00f, 0);
    else if (t < 4)  setKecepatan(0.15f, 0);
    else if (t < 7)  setKecepatan(0.30f, 0);
    else if (t < 10) setKecepatan(0.00f, 0);
    else { stopSemua(); Serial.println("# uji step selesai"); }
  } else if (mode == UJI_RAMP) {
    if (t < 1)       setKecepatan(0, 0);
    else if (t < 5)  setKecepatan(0.075f * (t - 1), 0);   // ramp 0,075 m/s^2 sampai 0,30 m/s
    else if (t < 7)  setKecepatan(0.30f, 0);
    else if (t < 9)  setKecepatan(0, 0);
    else { stopSemua(); Serial.println("# uji ramp selesai"); }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(20);
  pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT); pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT); pinMode(PIN_PWMB, OUTPUT);
  tulisMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0, 1);
  tulisMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, 0, 1);

  // Encoder JGA25-370 dicatu 3V3, keluaran open-collector -> pull-up internal
  pinMode(ENC_KI_A, INPUT_PULLUP); pinMode(ENC_KI_B, INPUT_PULLUP);
  pinMode(ENC_KA_A, INPUT_PULLUP); pinMode(ENC_KA_B, INPUT_PULLUP);
  stKi = (digitalRead(ENC_KI_A) << 1) | digitalRead(ENC_KI_B);
  stKa = (digitalRead(ENC_KA_A) << 1) | digitalRead(ENC_KA_B);
  attachInterrupt(digitalPinToInterrupt(ENC_KI_A), isrKi, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KI_B), isrKi, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KA_A), isrKa, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_KA_B), isrKa, CHANGE);

  Serial.println("# Uji PID Kecepatan Roda v2. Ketik ? untuk daftar perintah.");
  Serial.println("t_ms,refKi,refKa,vKi,vKa,uKi,uKa,x_m,y_m,th_deg");
  tPrev = tStart = millis();
}

void loop() {
  bacaSerial();
  unsigned long now = millis();
  if (now - tPrev < TS_MS) return;
  tPrev += TS_MS;

  if (mode != MANUAL && !fault) jalankanUji();

  // 1) ukur kecepatan (tick 1x per 50 ms, bertanda)
  noInterrupts(); long cKi = cntKi, cKa = cntKa; interrupts();
  float dKi = SIGN_ENC_KI * (cKi - lastKi) / 4.0f;
  float dKa = SIGN_ENC_KA * (cKa - lastKa) / 4.0f;
  lastKi = cKi; lastKa = cKa;
  ki.v = dKi; ka.v = dKa;

  // 2) odometri RK2 (titik tengah) di frame dunia
  float sL = dKi * M_PER_TICK, sR = dKa * M_PER_TICK;
  float ds = (sR + sL) / 2.0f, dth = (sR - sL) / TRACK_L;
  odoX += ds * cosf(odoTh + dth / 2.0f);
  odoY += ds * sinf(odoTh + dth / 2.0f);
  odoTh += dth;

  // 3) profil akselerasi + PID tiap roda
  ki.refRamp = rampTarget(ki.refRamp, ki.ref);
  ka.refRamp = rampTarget(ka.refRamp, ka.ref);
  pid(ki); pid(ka);
  if (!fault) {
    if (!cekPengaman(ki, gKi, "KIRI", "KI", now)) cekPengaman(ka, gKa, "KANAN", "KA", now);
  }
  if (fault) {   // fault terkunci: paksa keluaran nol
    ki.ref = ka.ref = ki.refRamp = ka.refRamp = 0;
    ki.u = ka.u = 0;
    ki.integ = ka.integ = 0;
  }
  tulisMotor(PIN_AIN1, PIN_AIN2, PIN_PWMA, ki.u, SIGN_MOT_KI);
  tulisMotor(PIN_BIN1, PIN_BIN2, PIN_PWMB, ka.u, SIGN_MOT_KA);

  // 4) log CSV (bisa dibuka di Serial Plotter atau disalin ke Excel/Python)
  if (logCsv) {
    Serial.printf("%lu,%.2f,%.2f,%.2f,%.2f,%.0f,%.0f,%.4f,%.4f,%.2f\n",
                  now - tStart, ki.refRamp, ka.refRamp, ki.v, ka.v, ki.u, ka.u,
                  odoX, odoY, odoTh * 180.0f / PI);
  }
}
