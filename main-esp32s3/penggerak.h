// penggerak.h - motor TB6612, encoder kuadratur 4x, PID kecepatan roda, profil trapesium,
// odometri RK2 dengan fusi heading gyro, pengaman stall/tanda salah (dari Uji_PID_Kecepatan_Roda_v2).
#pragma once
#include <Arduino.h>

enum BatasGerak : uint8_t {
  BATAS_TIDAK = 0,     // bebas
  BATAS_LARANG_MAJU,   // tebing: hanya boleh mundur lurus (v < 0, w = 0)
  BATAS_REM,           // berhenti dengan rem darurat (rampa A_REM), perintah baru diabaikan
  BATAS_MATI           // PWM 0 seketika (coast): fault, robot terangkat/terguling
};

struct Pose {
  float x, y, th;   // m, m, rad (frame dunia {W} = pose saat reset / START)
  float s;          // jarak tempuh bertanda sepanjang lintasan (m)
};

void penggerakMulai();
void penggerakUpdate(uint32_t nowMs);            // panggil tiap TS_MS (50 ms)

void penggerakSetKecepatan(float vMps, float wRadps);
void penggerakBerhenti(bool darurat);             // target 0 dengan rampa normal atau rem darurat
void penggerakSetBatas(BatasGerak b);
BatasGerak penggerakBatas();

// Mode uji: PWM mentah (melewati PID) dengan berhenti otomatis setelah durasi
void penggerakPwmMentah(int pwmKi, int pwmKa, uint32_t durasiMs);
bool penggerakModeMentah();

bool penggerakFault();
const char *penggerakAlasanFault();
void penggerakLepasFault();

Pose penggerakPose();
void penggerakResetOdo();
float penggerakV();                // m/s terukur
float penggerakW();                // rad/s terukur (fusi)
bool penggerakDiam();              // kedua roda diam dan target 0
void penggerakTick(long &kiri, long &kanan);   // hitungan 1x bertanda (sudah dikali SIGN_ENC)
void penggerakCetak(Stream &s);
