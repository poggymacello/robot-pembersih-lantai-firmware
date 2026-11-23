// sensor_jarak.h - HC-SR04 x3 (TRIG lewat PCF8574, ECHO interupsi) dan VL53L0X x2 (serong +-45 derajat)
#pragma once
#include <Arduino.h>

enum IdUs : uint8_t { US_DEPAN = 0, US_KIRI = 1, US_KANAN = 2, N_US = 3 };
enum IdTof : uint8_t { TOF_KI = 0, TOF_KA = 1, N_TOF = 2 };

// ---------- HC-SR04 ----------
void usMulai();
void usPoll();                      // panggil sesering mungkin dari loop() (non-blocking)
float usJarakCm(uint8_t id);        // US_JAUH_CM bila tidak ada gema; -1 bila data basi/sensor rusak
bool usValid(uint8_t id);
bool usRusak(uint8_t id);           // tidak ada respons berturut-turut / ECHO macet
void usSetAktif(bool aktif);        // hemat: matikan pemicuan saat diam

// ---------- VL53L0X ----------
// Urutan alamat (XSHUT) selalu dijalankan. Pengukuran jarak memakai pustaka Pololu "VL53L0X"
// bila PAKAI_PUSTAKA_VL53L0X = 1 di config.h (pustaka harus terpasang). Bila 0: STUB (tofTersedia() = false).
bool tofMulai(String &laporan);     // panggil saat boot (blokir singkat ~30 ms)
void tofPoll(uint32_t nowMs);
bool tofTersedia();                 // pustaka ada DAN kedua/salah satu sensor siap
bool tofPustakaAda();
int tofJarakMm(uint8_t id);         // TOF_JAUH_MM bila bebas, -1 bila tidak valid
bool tofValid(uint8_t id);

void jarakCetak(Stream &s);
