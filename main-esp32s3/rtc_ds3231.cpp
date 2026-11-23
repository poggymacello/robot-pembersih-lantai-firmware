// rtc_ds3231.cpp - register DS3231 (pola Bringup_PCB)
#include "rtc_ds3231.h"
#include "config.h"
#include "i2c_bus.h"
#include <Wire.h>

static bool sAda = false;

static uint8_t bcd2dec(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t dec2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool rtcMulai() {
  sAda = i2cAda(ADDR_RTC);
  return sAda;
}
bool rtcAda() { return sAda; }

bool rtcBaca(WaktuRtc &w) {
  uint8_t r[7];
  w.valid = false;
  if (!i2cBacaReg(ADDR_RTC, 0x00, r, 7)) { sAda = false; return false; }
  sAda = true;
  w.dtk = bcd2dec(r[0] & 0x7F);
  w.mnt = bcd2dec(r[1] & 0x7F);
  if (r[2] & 0x40) w.jam = bcd2dec(r[2] & 0x1F) % 12 + ((r[2] & 0x20) ? 12 : 0);
  else w.jam = bcd2dec(r[2] & 0x3F);
  w.hari = r[3] & 0x07;
  w.tgl = bcd2dec(r[4] & 0x3F);
  w.bln = bcd2dec(r[5] & 0x1F);
  w.thn = 2000 + bcd2dec(r[6]) + ((r[5] & 0x80) ? 100 : 0);
  int status = i2cBaca1(ADDR_RTC, 0x0F);
  w.valid = status >= 0 && !(status & 0x80);   // OSF = 1 -> waktu tidak valid
  return true;
}

bool rtcSet(const WaktuRtc &w) {
  static const int tt[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
  int y = w.thn - (w.bln < 3);
  int hari = (y + y / 4 - y / 100 + y / 400 + tt[(w.bln - 1) % 12] + w.tgl) % 7;   // 0 = Minggu
  Wire.beginTransmission(ADDR_RTC);
  Wire.write(0x00);
  Wire.write(dec2bcd(w.dtk));
  Wire.write(dec2bcd(w.mnt));
  Wire.write(dec2bcd(w.jam));     // mode 24 jam
  Wire.write(hari + 1);
  Wire.write(dec2bcd(w.tgl));
  Wire.write(dec2bcd(w.bln));
  Wire.write(dec2bcd(w.thn % 100));
  bool ok = Wire.endTransmission() == 0;
  int status = i2cBaca1(ADDR_RTC, 0x0F);
  if (status >= 0) i2cTulisReg(ADDR_RTC, 0x0F, status & ~0x80);   // hapus OSF
  return ok;
}

void rtcCetak(Stream &s, const WaktuRtc &w) {
  s.printf("%04u-%02u-%02u %02u:%02u:%02u%s", w.thn, w.bln, w.tgl, w.jam, w.mnt, w.dtk,
           w.valid ? "" : " (TIDAK VALID: OSF=1, set dengan 'waktu set')");
}
