// keselamatan.cpp
#include "keselamatan.h"
#include "config.h"
#include "param.h"
#include "penggerak.h"
#include "sensor_imu.h"
#include "sensor_jarak.h"
#include "daya.h"
#include "kamera_link.h"

static uint8_t sHitKi = 0, sHitKa = 0;
static bool sTebingKi = false, sTebingKa = false;
static bool sMiring = false;
static uint32_t sTMiring = 0;
static uint16_t sKondisi = 0;
static bool sAbaikanTebing = false;

void amanMulai() {
  pinMode(PIN_TCRT_KI, INPUT);
  pinMode(PIN_TCRT_KA, INPUT);
}

void amanAbaikanTebing(bool abaikan) { sAbaikanTebing = abaikan; }
bool amanTebingDiabaikan() { return sAbaikanTebing; }

void amanUpdate(uint32_t now) {
  // --- TCRT5000 (debounce TCRT_DEBOUNCE sampel) ---
  bool rKi = digitalRead(PIN_TCRT_KI) == TCRT_LEVEL_TEBING;
  bool rKa = digitalRead(PIN_TCRT_KA) == TCRT_LEVEL_TEBING;
  sHitKi = rKi ? (sHitKi < 255 ? sHitKi + 1 : 255) : 0;
  sHitKa = rKa ? (sHitKa < 255 ? sHitKa + 1 : 255) : 0;
  sTebingKi = sHitKi >= TCRT_DEBOUNCE;
  sTebingKa = sHitKa >= TCRT_DEBOUNCE;

  // --- kemiringan (robot terangkat / terguling) ---
  bool mBaru = imuOk() && imuTiltDeg() > P.tiltMaks;
  if (mBaru) { if (!sTMiring) sTMiring = now; sMiring = (now - sTMiring) > TILT_TAHAN_MS; }
  else { sTMiring = 0; sMiring = false; }

  // --- kumpulkan kondisi ---
  uint16_t k = 0;
  if (sTebingKi) k |= KA_TEBING_KI;
  if (sTebingKa) k |= KA_TEBING_KA;
  if (sMiring) k |= KA_MIRING;
  if (penggerakFault()) k |= KA_FAULT_RODA;
  if (dayaStatus() == BAT_KRITIS) k |= KA_BAT_KRITIS;
  if (dayaStatus() == BAT_PERINGATAN) k |= KA_BAT_WARN;
  if (!kameraAda()) k |= KA_KAMERA_ABSEN;
  if (usRusak(US_DEPAN)) k |= KA_US_DEPAN_RUSAK;
  if (!imuOk()) k |= KA_IMU_GAGAL;
  sKondisi = k;

  // --- terapkan (menimpa apa pun keputusan FSM) ---
  BatasGerak b = BATAS_TIDAK;
  if (k & (KA_MIRING | KA_FAULT_RODA)) b = BATAS_MATI;
  else if (k & KA_BAT_KRITIS) b = BATAS_REM;
  else if (amanTebing()) b = BATAS_LARANG_MAJU;
  if (penggerakBatas() != b) penggerakSetBatas(b);
  if ((k & (KA_MIRING | KA_FAULT_RODA | KA_BAT_KRITIS)) && suctionNyala()) suctionSet(false);
}

uint16_t amanKondisi() { return sKondisi; }
bool amanTebingKi() { return sTebingKi && !sAbaikanTebing; }
bool amanTebingKa() { return sTebingKa && !sAbaikanTebing; }
bool amanTebing() { return amanTebingKi() || amanTebingKa(); }
bool amanMiring() { return sMiring; }

void amanCetak(Stream &s) {
  s.printf("# Keselamatan: tebing KI=%d KA=%d%s, tilt %.1f deg (batas %.0f)%s, fault roda %d, baterai %s %.2f V, kondisi 0x%04X\n",
           sTebingKi, sTebingKa, sAbaikanTebing ? " (DIABAIKAN, mode uji)" : "", imuTiltDeg(), P.tiltMaks,
           sMiring ? " MIRING" : "", penggerakFault(), dayaNamaStatus(), dayaVbat(), sKondisi);
}
