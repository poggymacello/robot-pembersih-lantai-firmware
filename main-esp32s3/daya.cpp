// daya.cpp - baterai dan suction
#include "daya.h"
#include "config.h"
#include "param.h"

static float sVf = 0, sVs = 0;
static bool sAwal = true;
static StatusBaterai sSt = BAT_NORMAL;
static uint32_t sTBawahWarn = 0, sTAtasWarn = 0, sTBawahStop = 0, sTAtasPulih = 0;
static bool sSuction = false;
static uint32_t sTSuction = 0;

void dayaMulai() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
  pinMode(PIN_SUCTION, OUTPUT);
  digitalWrite(PIN_SUCTION, LOW);
}

static float bacaVbat() {
  uint32_t jml = 0;
  const int N = 16;
  for (int i = 0; i < N; i++) jml += analogReadMilliVolts(PIN_VBAT);
  return (jml / (float)N) * VBAT_RASIO * P.vbatKoreksi / 1000.0f;
}

void dayaUpdate(uint32_t now) {
  sVs = bacaVbat();
  if (sAwal) { sVf = sVs; sAwal = false; }
  sVf += 0.2f * (sVs - sVf);    // EMA, tau ~1,1 s pada 4 Hz

  if (sVf < VBAT_ABSEN_V) {     // sense tidak tersambung (uji meja lewat USB): tanpa cutoff
    sSt = BAT_ABSEN;
    sTBawahWarn = sTBawahStop = sTAtasPulih = 0;
    return;
  }
  if (sSt == BAT_ABSEN) sSt = BAT_NORMAL;

  // KRITIS terkunci: lepas hanya bila tegangan (tanpa beban) > vpulih selama VBAT_TAHAN_PULIH_MS
  if (sSt == BAT_KRITIS) {
    if (sVf > P.vbatPulih) {
      if (!sTAtasPulih) sTAtasPulih = now;
      if (now - sTAtasPulih > VBAT_TAHAN_PULIH_MS) { sSt = BAT_NORMAL; sTAtasPulih = 0; sTBawahStop = 0; }
    } else sTAtasPulih = 0;
    return;
  }

  if (sVf < P.vbatStop) {
    if (!sTBawahStop) sTBawahStop = now;
    if (now - sTBawahStop > VBAT_TAHAN_STOP_MS) { sSt = BAT_KRITIS; return; }
  } else sTBawahStop = 0;

  if (sVf < P.vbatPeringatan) {
    sTAtasWarn = 0;
    if (!sTBawahWarn) sTBawahWarn = now;
    if (now - sTBawahWarn > VBAT_TAHAN_PERINGATAN_MS) sSt = BAT_PERINGATAN;
  } else {
    sTBawahWarn = 0;
    if (sSt == BAT_PERINGATAN && sVf > P.vbatPeringatan + VBAT_HISTERESIS) {
      if (!sTAtasWarn) sTAtasWarn = now;
      if (now - sTAtasWarn > VBAT_TAHAN_PERINGATAN_MS) { sSt = BAT_NORMAL; sTAtasWarn = 0; }
    } else sTAtasWarn = 0;
  }
}

float dayaVbat() { return sVf; }
float dayaVbatSesaat() { return sVs; }
StatusBaterai dayaStatus() { return sSt; }
const char *dayaNamaStatus() {
  switch (sSt) {
    case BAT_NORMAL: return "NORMAL";
    case BAT_PERINGATAN: return "PERINGATAN";
    case BAT_KRITIS: return "KRITIS";
    default: return "ABSEN (sense tidak tersambung)";
  }
}

void suctionSet(bool nyala) {
  if (nyala && !sSuction) sTSuction = millis();
  sSuction = nyala;
  digitalWrite(PIN_SUCTION, nyala ? HIGH : LOW);
}
bool suctionNyala() { return sSuction; }
uint32_t suctionLamaNyalaMs() { return sSuction ? millis() - sTSuction : 0; }
