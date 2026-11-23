// sensor_jarak.cpp - HC-SR04 berurutan non-blocking + VL53L0X
#include "sensor_jarak.h"
#include "config.h"
#include "i2c_bus.h"
#include <Wire.h>

// =====================================================================
//  HC-SR04
//  Urutan slot: DEPAN, KIRI, DEPAN, KANAN -> sensor depan diukur 2x lebih sering.
//  TRIG: bit PCF8574 HIGH lalu LOW (tiap transaksi I2C 100 kHz ~0,2 ms > 10 us).
//  ECHO: interupsi CHANGE mencatat micros() naik/turun. Tidak ada pulseIn (tidak memblokir).
// =====================================================================
static const uint8_t PIN_ECHO[N_US] = {PIN_ECHO_US1, PIN_ECHO_US4, PIN_ECHO_US5};
static const uint8_t BIT_TRIG[N_US] = {PCF_TRIG_US1, PCF_TRIG_US4, PCF_TRIG_US5};
static const uint8_t URUTAN[] = {US_DEPAN, US_KIRI, US_DEPAN, US_KANAN};
static const uint8_t N_URUTAN = sizeof(URUTAN);

static volatile uint32_t sNaikUs[N_US], sTurunUs[N_US];
static volatile uint8_t sEdge[N_US];   // bit0 = naik tercatat, bit1 = turun tercatat

static float sJarak[N_US] = {US_JAUH_CM, US_JAUH_CM, US_JAUH_CM};
static uint32_t sTData[N_US] = {0, 0, 0};
static uint8_t sGagal[N_US] = {0, 0, 0};
static bool sMacet[N_US] = {false, false, false};
static bool sAktif = true;

enum StUs : uint8_t { US_SIAP, US_TUNGGU, US_JEDA };
static StUs sSt = US_SIAP;
static uint8_t sSlot = 0;
static uint32_t sTTrigUs = 0, sTJedaMs = 0, sTMacetMs = 0;

template <uint8_t I, uint8_t PIN>
static void IRAM_ATTR isrEcho() {
  uint32_t t = micros();
  if (digitalRead(PIN)) { sNaikUs[I] = t; sEdge[I] = 1; }
  else if (sEdge[I] & 1) { sTurunUs[I] = t; sEdge[I] |= 2; }
}

void usMulai() {
  for (uint8_t i = 0; i < N_US; i++) { pinMode(PIN_ECHO[i], INPUT); sEdge[i] = 0; }
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_US1), isrEcho<0, PIN_ECHO_US1>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_US4), isrEcho<1, PIN_ECHO_US4>, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO_US5), isrEcho<2, PIN_ECHO_US5>, CHANGE);
}

void usSetAktif(bool aktif) { sAktif = aktif; }

static void simpanHasil(uint8_t id, float cm, bool respons) {
  if (respons) { sGagal[id] = 0; sJarak[id] = cm; sTData[id] = millis(); }
  else if (sGagal[id] < 255) sGagal[id]++;
}

void usPoll() {
  if (!pcfAda()) return;
  uint8_t id = URUTAN[sSlot];
  uint32_t nowMs = millis();
  switch (sSt) {
    case US_SIAP:
      if (!sAktif) return;
      if (digitalRead(PIN_ECHO[id]) == HIGH) {          // masih sibuk dari ukur sebelumnya / macet
        if (sTMacetMs == 0) sTMacetMs = nowMs;
        if (nowMs - sTMacetMs > US_ECHO_MACET_MS) {
          sMacet[id] = true;
          simpanHasil(id, 0, false);
          sTMacetMs = 0;
          sSlot = (sSlot + 1) % N_URUTAN;
        }
        return;
      }
      sTMacetMs = 0;
      sMacet[id] = false;
      noInterrupts(); sEdge[id] = 0; interrupts();
      pcfSetBit(BIT_TRIG[id], true);
      pcfSetBit(BIT_TRIG[id], false);                     // tepi turun TRIG -> burst dikirim
      sTTrigUs = micros();
      sSt = US_TUNGGU;
      break;

    case US_TUNGGU: {
      uint8_t e;
      uint32_t naik, turun;
      noInterrupts(); e = sEdge[id]; naik = sNaikUs[id]; turun = sTurunUs[id]; interrupts();
      if (e & 2) {
        float cm = (turun - naik) / 58.0f;
        simpanHasil(id, cm > US_JAUH_CM ? US_JAUH_CM : cm, true);
        sSt = US_JEDA; sTJedaMs = nowMs;
      } else if (micros() - sTTrigUs > US_TIMEOUT_US) {
        // gema naik tetapi tidak turun dalam batas waktu = tidak ada pantulan dalam jangkauan (bebas)
        if (e & 1) simpanHasil(id, US_JAUH_CM, true);
        else simpanHasil(id, 0, false);                   // ECHO tidak pernah naik -> tidak ada respons
        sSt = US_JEDA; sTJedaMs = nowMs;
      }
      break;
    }

    case US_JEDA:
      if (nowMs - sTJedaMs >= US_JEDA_MS) {
        sSt = US_SIAP;
        sSlot = (sSlot + 1) % N_URUTAN;
      }
      break;
  }
}

bool usRusak(uint8_t id) { return sMacet[id] || sGagal[id] >= US_GAGAL_MAKS || !pcfAda(); }
bool usValid(uint8_t id) { return !usRusak(id) && sTData[id] != 0 && millis() - sTData[id] < US_DATA_BASI_MS; }
float usJarakCm(uint8_t id) { return usValid(id) ? sJarak[id] : -1.0f; }

// =====================================================================
//  VL53L0X
// =====================================================================
#if PAKAI_PUSTAKA_VL53L0X
  #include <VL53L0X.h>        // pustaka Pololu "VL53L0X" (Library Manager: "VL53L0X by Pololu")
  #define TOF_PUSTAKA 1
  static VL53L0X sTof[N_TOF];
#else
  // ------------------------------------------------------------------
  //  STUB: PAKAI_PUSTAKA_VL53L0X = 0 (config.h). Pustaka Pololu VL53L0X belum terpasang.
  //  Alamat 0x30/0x31 tetap diatur (pemindai I2C lengkap), tetapi jarak TIDAK diukur:
  //  tofTersedia() = false dan navigasi hanya memakai HC-SR04.
  //  Cara mengaktifkan: Arduino IDE > Library Manager > cari "VL53L0X" > pasang
  //  "VL53L0X by Pololu" > ubah PAKAI_PUSTAKA_VL53L0X menjadi 1 di config.h > kompilasi ulang.
  // ------------------------------------------------------------------
  #define TOF_PUSTAKA 0
#endif

static const uint8_t PIN_XSHUT[N_TOF] = {PIN_XSHUT_KI, PIN_XSHUT_KA};
static const uint8_t ADDR_TOF[N_TOF] = {ADDR_TOF_KI, ADDR_TOF_KA};
static bool sTofSiap[N_TOF] = {false, false};
static int sTofMm[N_TOF] = {-1, -1};
static uint32_t sTofT[N_TOF] = {0, 0};
static uint32_t sTofPoll = 0;

bool tofPustakaAda() { return TOF_PUSTAKA; }

bool tofMulai(String &laporan) {
  laporan = "";
  for (uint8_t i = 0; i < N_TOF; i++) { pinMode(PIN_XSHUT[i], OUTPUT); digitalWrite(PIN_XSHUT[i], LOW); }
  delay(10);
  bool semuaAlamat = true;
  for (uint8_t i = 0; i < N_TOF; i++) {
    const char *nama = i == TOF_KI ? "TOF-KI" : "TOF-KA";
    digitalWrite(PIN_XSHUT[i], HIGH);   // tetap OUTPUT (pull-down di PCB)
    delay(10);
    bool alamatOk = false;
    if (i2cAda(ADDR_TOF_BAWAAN) && i2cTulisReg(ADDR_TOF_BAWAAN, 0x8A, ADDR_TOF[i] & 0x7F)) {
      delay(2);
      alamatOk = i2cAda(ADDR_TOF[i]) && i2cBaca1(ADDR_TOF[i], 0xC0) == 0xEE;
    } else if (i2cAda(ADDR_TOF[i])) {
      alamatOk = true;   // sudah di alamat baru (mis. reset ESP32 tanpa sensor kehilangan daya)
    }
    if (!alamatOk) { semuaAlamat = false; laporan += String(nama) + " alamat GAGAL; "; continue; }
#if TOF_PUSTAKA
    sTof[i].setBus(&Wire);
    sTof[i].setAddress(ADDR_TOF[i]);   // alamat sudah diubah, ini hanya menyamakan variabel pustaka
    sTof[i].setTimeout(30);
    if (sTof[i].init()) {
      sTof[i].setMeasurementTimingBudget(33000);
      sTof[i].startContinuous(0);       // back-to-back, dibaca non-blocking di tofPoll()
      sTofSiap[i] = true;
      laporan += String(nama) + " OK; ";
    } else {
      laporan += String(nama) + " init GAGAL; ";
    }
#else
    laporan += String(nama) + " alamat OK (jarak: STUB, pustaka tidak ada); ";
#endif
  }
  return semuaAlamat;
}

void tofPoll(uint32_t nowMs) {
#if TOF_PUSTAKA
  if (nowMs - sTofPoll < TOF_PERIODE_MS / 2) return;
  sTofPoll = nowMs;
  for (uint8_t i = 0; i < N_TOF; i++) {
    if (!sTofSiap[i]) continue;
    // non-blocking: baca hanya bila data siap (RESULT_INTERRUPT_STATUS bit 0..2)
    if ((sTof[i].readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) continue;
    uint16_t mm = sTof[i].readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
    sTof[i].writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    sTofMm[i] = (mm == 0 || mm >= 8190 || mm > TOF_JAUH_MM) ? TOF_JAUH_MM : mm;
    sTofT[i] = nowMs;
  }
#else
  (void)nowMs; (void)sTofPoll;   // STUB: tidak ada pengukuran
#endif
}

bool tofTersedia() { return TOF_PUSTAKA && (sTofSiap[0] || sTofSiap[1]); }
bool tofValid(uint8_t id) { return sTofSiap[id] && sTofT[id] != 0 && millis() - sTofT[id] < 300; }
int tofJarakMm(uint8_t id) { return tofValid(id) ? sTofMm[id] : -1; }

void jarakCetak(Stream &s) {
  static const char *NAMA[N_US] = {"depan", "kiri", "kanan"};
  s.print(F("# HC-SR04 [cm]:"));
  for (uint8_t i = 0; i < N_US; i++) {
    if (usRusak(i)) s.printf(" %s=RUSAK%s", NAMA[i], sMacet[i] ? "(ECHO macet)" : "");
    else s.printf(" %s=%.1f", NAMA[i], usJarakCm(i));
  }
  s.printf("  (PCF8574 %s, gagal tulis %lu)\n", pcfAda() ? "ada" : "TIDAK ADA", (unsigned long)pcfJumlahGagal());
  if (!TOF_PUSTAKA) {
    s.println(F("# VL53L0X: STUB (PAKAI_PUSTAKA_VL53L0X = 0 di config.h), jarak tidak diukur."));
  } else {
    s.printf("# VL53L0X [mm]: KI=%d%s KA=%d%s\n", tofJarakMm(0), sTofSiap[0] ? "" : "(tidak siap)",
             tofJarakMm(1), sTofSiap[1] ? "" : "(tidak siap)");
  }
}
