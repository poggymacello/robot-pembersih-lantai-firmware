// indikator.h - LED RGB (katoda bersama) dan buzzer aktif, pola non-blocking
#pragma once
#include <Arduino.h>

enum PolaLed : uint8_t {
  LED_MATI = 0,
  LED_BOOT,          // biru kedip cepat
  LED_SIAGA,         // hijau tetap (IDLE)
  LED_BERSIH,        // hijau kedip 1 Hz
  LED_KOTOR,         // cyan tetap (kamera melihat kotor)
  LED_HINDAR,        // kuning kedip cepat (AVOID / CLIFF_ESCAPE)
  LED_PERINGATAN,    // kuning tetap (baterai peringatan, saat IDLE)
  LED_BATERAI,       // merah kedip (LOW_BATTERY_STOP)
  LED_GALAT,         // merah tetap (ERROR)
  LED_UJI            // magenta tetap (mode uji CLI)
};

void indikatorMulai();
void indikatorUpdate(uint32_t nowMs);
void ledSetPola(PolaLed p);
void ledSetWarna(bool r, bool g, bool b);   // langsung (uji), menimpa pola sampai ledSetPola dipanggil lagi

// Buzzer: n bunyi (on/off ms); bila periodeMs > 0 pola diulang tiap periodeMs (alarm).
void buzzerPola(uint8_t n, uint16_t onMs, uint16_t offMs, uint32_t periodeMs = 0);
void buzzerDiam();
