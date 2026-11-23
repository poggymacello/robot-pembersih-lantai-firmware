// fsm.h - mesin keadaan utama robot (outer loop 10 Hz)
//
//   BOOT/SELFTEST -> IDLE -> CLEANING (BERSIH) <-> AVOID (HINDAR)
//                              |  \-> CLIFF_ESCAPE (TEBING)
//                              v
//          LOW_BATTERY_STOP (BATERAI) / ERROR (GALAT) / UJI (perintah CLI)
#pragma once
#include <Arduino.h>

enum StateRobot : uint8_t {
  ST_BOOT = 0,   // selftest + estimasi bias gyro (robot diam)
  ST_IDLE,       // menunggu "mulai" (CLI) atau jadwal DS3231
  ST_BERSIH,     // CLEANING: cakupan bolak-balik (boustrophedon) dengan heading gyro
  ST_HINDAR,     // AVOID: mundur lalu belok-U ke sisi yang lebih lega
  ST_TEBING,     // CLIFF_ESCAPE: rem, mundur, putar menjauhi tebing
  ST_BATERAI,    // LOW_BATTERY_STOP: semua aman + alarm, tunggu diisi
  ST_GALAT,      // ERROR: semua aman + alarm, keluar hanya dengan "reset"
  ST_UJI         // mode uji dari CLI (gerak manual, suction, dsb.)
};

void fsmMulai();
void fsmUpdate(uint32_t nowMs);              // panggil tiap 100 ms
void fsmTiapDetik(uint32_t nowMs);           // jadwal RTC, dll.
StateRobot fsmState();
const char *fsmNamaState();
const char *fsmNamaSub();
const char *fsmAlasanGalat();

bool fsmPerintahMulai(const char *sumber, String &pesan);   // IDLE -> BERSIH
void fsmPerintahBerhenti(const char *alasan);                // apa pun -> IDLE (kecuali GALAT/BATERAI)
bool fsmPerintahReset(String &pesan);                        // GALAT -> IDLE bila penyebab sudah hilang
void fsmGalat(const char *alasan);

// Mode uji (hanya dari IDLE / UJI)
bool fsmUjiMasuk(String &pesan);
bool fsmUjiJalan(float v, float w, uint32_t durasiMs, String &pesan);
bool fsmUjiPutar(float derajat, String &pesan);
bool fsmUjiLurus(float meter, String &pesan);
void fsmCetak(Stream &s);
