// keselamatan.h - supervisor keselamatan. Berjalan tiap 10 ms SEBELUM dan SESUDAH FSM,
// dan memaksa batas gerak/suction tanpa bergantung pada FSM:
//   miring/terangkat atau fault roda -> PWM 0 seketika + suction mati
//   baterai kritis                   -> rem darurat + suction mati
//   tebing (TCRT)                    -> larang maju (hanya mundur lurus)
#pragma once
#include <Arduino.h>

enum KondisiAman : uint16_t {
  KA_TEBING_KI   = 0x0001,
  KA_TEBING_KA   = 0x0002,
  KA_MIRING      = 0x0004,
  KA_FAULT_RODA  = 0x0008,
  KA_BAT_KRITIS  = 0x0010,
  KA_BAT_WARN    = 0x0020,
  KA_KAMERA_ABSEN= 0x0040,
  KA_US_DEPAN_RUSAK = 0x0080,
  KA_IMU_GAGAL   = 0x0100,
};

void amanMulai();
void amanUpdate(uint32_t nowMs);     // baca TCRT (debounce), kemiringan, kumpulkan kondisi, terapkan batas
uint16_t amanKondisi();
bool amanTebing();
bool amanTebingKi();
bool amanTebingKa();
bool amanMiring();
// Uji di meja (roda diangkat, TCRT tidak melihat lantai): abaikan tebing. Otomatis kembali aktif saat keluar mode UJI.
void amanAbaikanTebing(bool abaikan);
bool amanTebingDiabaikan();
void amanCetak(Stream &s);
