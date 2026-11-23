// rtc_ds3231.h - DS3231 (0x68) baca/set waktu, untuk jadwal mulai otomatis
#pragma once
#include <Arduino.h>

struct WaktuRtc {
  uint16_t thn; uint8_t bln, tgl, jam, mnt, dtk, hari;   // hari 1 = Minggu
  bool valid;      // terbaca dan flag OSF = 0
};

bool rtcMulai();                  // true bila DS3231 menjawab
bool rtcBaca(WaktuRtc &w);
bool rtcSet(const WaktuRtc &w);   // hari dihitung otomatis
bool rtcAda();
void rtcCetak(Stream &s, const WaktuRtc &w);
