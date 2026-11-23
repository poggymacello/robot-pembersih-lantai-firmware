// indikator.cpp
#include "indikator.h"
#include "config.h"

static PolaLed sPola = LED_MATI;
static bool sManual = false;
// buzzer
static uint8_t sBzN = 0, sBzSisa = 0;
static uint16_t sBzOn = 0, sBzOff = 0;
static uint32_t sBzPeriode = 0, sBzTMulaiPola = 0, sBzTFase = 0;
static bool sBzNyala = false;

static void tulisLed(bool r, bool g, bool b) {
  digitalWrite(PIN_LED_R, (r == LED_AKTIF_HIGH) ? HIGH : LOW);
  digitalWrite(PIN_LED_G, (g == LED_AKTIF_HIGH) ? HIGH : LOW);
  digitalWrite(PIN_LED_B, (b == LED_AKTIF_HIGH) ? HIGH : LOW);
}

void indikatorMulai() {
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  tulisLed(false, false, false);
}

void ledSetPola(PolaLed p) { sPola = p; sManual = false; }
void ledSetWarna(bool r, bool g, bool b) { sManual = true; tulisLed(r, g, b); }

void buzzerPola(uint8_t n, uint16_t onMs, uint16_t offMs, uint32_t periodeMs) {
  sBzN = n; sBzSisa = n; sBzOn = onMs; sBzOff = offMs; sBzPeriode = periodeMs;
  sBzTMulaiPola = sBzTFase = millis();
  sBzNyala = n > 0;
  digitalWrite(PIN_BUZZER, sBzNyala ? HIGH : LOW);
}

void buzzerDiam() {
  sBzN = sBzSisa = 0; sBzPeriode = 0; sBzNyala = false;
  digitalWrite(PIN_BUZZER, LOW);
}

static void updateBuzzer(uint32_t now) {
  if (sBzSisa == 0) {
    if (sBzPeriode && sBzN && now - sBzTMulaiPola >= sBzPeriode) {
      sBzSisa = sBzN; sBzTMulaiPola = sBzTFase = now; sBzNyala = true;
      digitalWrite(PIN_BUZZER, HIGH);
    }
    return;
  }
  if (sBzNyala && now - sBzTFase >= sBzOn) {
    sBzNyala = false; sBzTFase = now; digitalWrite(PIN_BUZZER, LOW);
    sBzSisa--;
  } else if (!sBzNyala && sBzSisa > 0 && now - sBzTFase >= sBzOff) {
    sBzNyala = true; sBzTFase = now; digitalWrite(PIN_BUZZER, HIGH);
  }
}

void indikatorUpdate(uint32_t now) {
  updateBuzzer(now);
  if (sManual) return;
  bool k1 = (now / 500) & 1;    // 1 Hz
  bool k4 = (now / 125) & 1;    // 4 Hz
  switch (sPola) {
    case LED_MATI:       tulisLed(0, 0, 0); break;
    case LED_BOOT:       tulisLed(0, 0, k4); break;
    case LED_SIAGA:      tulisLed(0, 1, 0); break;
    case LED_BERSIH:     tulisLed(0, k1, 0); break;
    case LED_KOTOR:      tulisLed(0, 1, 1); break;
    case LED_HINDAR:     tulisLed(k4, k4, 0); break;
    case LED_PERINGATAN: tulisLed(1, 1, 0); break;   // kuning tetap
    case LED_BATERAI:    tulisLed(k1, 0, 0); break;
    case LED_GALAT:      tulisLed(1, 0, 0); break;
    case LED_UJI:        tulisLed(1, 0, 1); break;
  }
}
