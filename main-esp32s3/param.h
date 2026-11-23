// param.h - Parameter yang bisa diubah saat berjalan (CLI "set <nama> <nilai>", "simpan")
#pragma once
#include <Arduino.h>

struct Param {
  // PID dan profil
  float kp, ki, kd;
  float aMaks, aRem;
  // Navigasi
  float vBersih, vKotor, vDekat, vMundur, wPutar, kHeading;
  float lebarLajur, lajurMaks, durasiMaksMnt, jarakKamera;
  float dDepanBelok, dDepanLambat, dDepanDarurat, dSamping, dTofBelok, dTofDarurat;
  float mundurTebing;
  // Kotor: 0 = abaikan kamera, 1 = perlambat, 2 = perlambat + ulangi lintasan sekali
  float modeKotor;
  float kotorPerluKalib;   // 1 = abaikan hasil kamera yang belum dikalibrasi
  // Suction saat membersihkan (1 = nyala)
  float suction;
  // Keselamatan / daya
  float tiltMaks, alphaGyro;
  float vbatKoreksi, vbatPeringatan, vbatStop, vbatPulih;
  // Jadwal DS3231
  float jadwalAktif, jadwalJam, jadwalMenit;
};

extern Param P;

void paramBawaan();
bool paramMuat();          // dari NVS namespace "robot"
bool paramSimpan();
bool paramSet(const char *nama, float nilai, String &pesan);
void paramCetak(Stream &s);
