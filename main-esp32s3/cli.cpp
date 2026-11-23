// cli.cpp - perintah serial
#include "cli.h"
#include "config.h"
#include "param.h"
#include "fsm.h"
#include "penggerak.h"
#include "sensor_jarak.h"
#include "sensor_imu.h"
#include "rtc_ds3231.h"
#include "daya.h"
#include "indikator.h"
#include "kamera_link.h"
#include "keselamatan.h"
#include "i2c_bus.h"
#include "protokol_kamera.h"

static char sBuf[100];
static uint8_t sLen = 0;
static bool sLog = false;
static uint32_t sLogPeriode = 200;
static uint32_t sSuctionSampai = 0;
static bool sUjiSuction = false;

bool cliLogAktif() { return sLog; }
uint32_t cliLogPeriode() { return sLogPeriode; }

static void bantuan() {
  Serial.println(F(
    "# ===== Perintah (huruf kecil, pisahkan dengan spasi) =====\n"
    "#  bantuan | ?                 daftar perintah\n"
    "#  status                      ringkasan state, sensor, baterai, kamera\n"
    "#  mulai                       mulai membersihkan (dari IDLE)\n"
    "#  berhenti | x                berhenti, kembali IDLE (motor & suction mati)\n"
    "#  reset                       lepas GALAT / fault roda setelah penyebab diperbaiki\n"
    "#  sensor                      cetak semua sensor sekali\n"
    "#  i2c                         pindai bus I2C\n"
    "#  log 0|1 [ms]                log baris CSV periodik (bawaan 200 ms)\n"
    "#  param | set <nama> <nilai> | simpan | bawaan | muat\n"
    "#  uji pwm <ki> <ka> <ms>      PWM mentah -255..255 (RODA DIANGKAT), maks 5000 ms\n"
    "#  uji jalan <v> <w> <ms>      kecepatan tertutup m/s, rad/s, maks 10000 ms\n"
    "#  uji putar <derajat>         putar di tempat dengan gyro (+ = kiri)\n"
    "#  uji lurus <meter>           maju (+) / mundur (-) lurus dengan heading gyro\n"
    "#  uji suction <ms>            suction ON maks 10000 ms\n"
    "#  uji buzzer | uji led\n"
    "#  uji tebing 0|1              1 = abaikan TCRT (roda diangkat di meja), hanya di mode UJI\n"
    "#  kamera [ping|mulai|berhenti|status|kalib|hapus|echo 0|1|set <id> <nilai>]\n"
    "#  waktu | waktu set YYYY-MM-DD HH:MM:SS\n"
    "#  jadwal HH:MM | jadwal off   mulai otomatis harian dari RTC (lalu 'simpan')\n"
    "#  gyro kalib                  perbarui bias gyro z (robot DIAM)\n"
    "#  odo reset                   nolkan pose\n"
    "#  versi"));
}

void cliCetakStatus() {
  Serial.printf("# ===== %s v%s, uptime %lu s =====\n", FW_NAMA, FW_VERSI, (unsigned long)(millis() / 1000));
  fsmCetak(Serial);
  penggerakCetak(Serial);
  if (penggerakFault()) Serial.printf("# fault roda: %s\n", penggerakAlasanFault());
  amanCetak(Serial);
  Serial.printf("# baterai: %.2f V (sesaat %.2f V) status %s | suction %s\n", dayaVbat(), dayaVbatSesaat(),
                dayaNamaStatus(), suctionNyala() ? "NYALA" : "mati");
  jarakCetak(Serial);
  imuCetak(Serial);
  kameraCetak(Serial);
  WaktuRtc w;
  if (rtcBaca(w)) { Serial.print(F("# RTC: ")); rtcCetak(Serial, w); Serial.println(); }
  else Serial.println(F("# RTC: tidak menjawab"));
  Serial.printf("# jadwal: %s %02d:%02d\n", P.jadwalAktif >= 0.5f ? "AKTIF" : "mati", (int)P.jadwalJam, (int)P.jadwalMenit);
}

void cliCetakLog() {
  Pose p = penggerakPose();
  const InfoKamera &k = kameraInfo();
  Serial.printf("%lu,%s,%s,%.3f,%.2f,%.3f,%.3f,%.1f,%.0f,%.0f,%.0f,%d,%d,%d%d,%.2f,%d,%u\n",
                (unsigned long)millis(), fsmNamaState(), fsmNamaSub(), penggerakV(), penggerakW(), p.x, p.y,
                p.th * 180.0f / PI, usJarakCm(US_DEPAN), usJarakCm(US_KIRI), usJarakCm(US_KANAN),
                tofJarakMm(TOF_KI), tofJarakMm(TOF_KA), amanTebingKi(), amanTebingKa(), dayaVbat(),
                kameraKotor() ? 1 : 0, kameraAda() ? k.skor : 0);
}

static int pecah(char *s, char **tok, int maks) {
  int n = 0;
  char *p = strtok(s, " \t");
  while (p && n < maks) { tok[n++] = p; p = strtok(nullptr, " \t"); }
  return n;
}

static void jawab(bool ok, const String &pesan) { Serial.printf("# %s%s\n", ok ? "" : "GAGAL: ", pesan.c_str()); }

static void perintahUji(char **t, int n) {
  String pesan;
  if (n < 2) { Serial.println(F("# uji pwm|jalan|putar|lurus|suction|buzzer|led|tebing")); return; }
  if (!strcmp(t[1], "pwm") && n >= 5) {
    if (!fsmUjiMasuk(pesan)) { jawab(false, pesan); return; }
    uint32_t ms = constrain(atol(t[4]), 0L, 5000L);
    penggerakPwmMentah(atoi(t[2]), atoi(t[3]), ms);
    Serial.printf("# PWM mentah ki %d ka %d selama %lu ms (tanpa PID, SIGN_MOT diterapkan)\n", atoi(t[2]), atoi(t[3]), (unsigned long)ms);
  } else if (!strcmp(t[1], "jalan") && n >= 5) {
    jawab(fsmUjiJalan(atof(t[2]), atof(t[3]), constrain(atol(t[4]), 0L, 10000L), pesan), pesan);
  } else if (!strcmp(t[1], "putar") && n >= 3) {
    jawab(fsmUjiPutar(atof(t[2]), pesan), pesan);
  } else if (!strcmp(t[1], "lurus") && n >= 3) {
    jawab(fsmUjiLurus(constrain((float)atof(t[2]), -2.0f, 5.0f), pesan), pesan);
  } else if (!strcmp(t[1], "suction") && n >= 3) {
    if (!fsmUjiMasuk(pesan)) { jawab(false, pesan); return; }
    uint32_t ms = constrain(atol(t[2]), 0L, 10000L);
    suctionSet(true);
    sUjiSuction = true;
    sSuctionSampai = millis() + ms;
    Serial.printf("# SUCTION NYALA %lu ms (ketik 'x' untuk berhenti). Pastikan saluran hisap bebas.\n", (unsigned long)ms);
  } else if (!strcmp(t[1], "buzzer")) {
    buzzerPola(3, 150, 150);
  } else if (!strcmp(t[1], "led")) {
    if (!fsmUjiMasuk(pesan)) { jawab(false, pesan); return; }
    static uint8_t w = 0;
    w = (w + 1) % 8;
    ledSetWarna(w & 1, w & 2, w & 4);
    Serial.printf("# LED R=%d G=%d B=%d (ulangi 'uji led' untuk warna berikutnya)\n", (w & 1) ? 1 : 0, (w & 2) ? 1 : 0, (w & 4) ? 1 : 0);
  } else if (!strcmp(t[1], "tebing") && n >= 3) {
    if (!fsmUjiMasuk(pesan)) { jawab(false, pesan); return; }
    amanAbaikanTebing(atoi(t[2]) != 0);
    Serial.printf("# sensor tebing %s (kembali aktif saat keluar mode UJI)\n", atoi(t[2]) ? "DIABAIKAN" : "aktif");
  } else {
    Serial.println(F("# argumen uji salah, ketik 'bantuan'"));
  }
}

static void perintahKamera(char **t, int n) {
  if (n < 2 || !strcmp(t[1], "status")) { kameraKirimMintaStatus(); kameraCetak(Serial); return; }
  if (!strcmp(t[1], "ping")) kameraKirimPing();
  else if (!strcmp(t[1], "mulai")) kameraKirimMulai(KAMERA_PERIODE_MS);
  else if (!strcmp(t[1], "berhenti")) kameraKirimBerhenti();
  else if (!strcmp(t[1], "kalib")) {
    kameraKirimKalibrasi();
    Serial.println(F("# kalibrasi kamera: robot DIAM di atas lantai BERSIH, ~6-10 s. Hasil muncul sebagai KAMERA STATUS."));
  } else if (!strcmp(t[1], "hapus")) kameraKirimHapusKalib();
  else if (!strcmp(t[1], "echo") && n >= 3) kameraSetEcho(atoi(t[2]) != 0);
  else if (!strcmp(t[1], "set") && n >= 4) kameraKirimSetParam((uint8_t)strtol(t[2], nullptr, 0), (int16_t)atoi(t[3]));
  else { Serial.println(F("# kamera [ping|mulai|berhenti|status|kalib|hapus|echo 0|1|set <id> <nilai>]")); return; }
  Serial.println(F("# terkirim"));
}

static void perintahWaktu(char **t, int n) {
  if (n >= 4 && !strcmp(t[1], "set")) {
    WaktuRtc w = {};
    int y, mo, d, h, mi, s;
    if (sscanf(t[2], "%d-%d-%d", &y, &mo, &d) == 3 && sscanf(t[3], "%d:%d:%d", &h, &mi, &s) == 3 &&
        y >= 2000 && y < 2100 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31 && h < 24 && mi < 60 && s < 60) {
      w.thn = y; w.bln = mo; w.tgl = d; w.jam = h; w.mnt = mi; w.dtk = s;
      Serial.println(rtcSet(w) ? F("# RTC diset") : F("# GAGAL set RTC"));
    } else Serial.println(F("# format: waktu set 2026-11-24 08:30:00"));
    return;
  }
  WaktuRtc w;
  if (rtcBaca(w)) { Serial.print(F("# RTC: ")); rtcCetak(Serial, w); Serial.println(); }
  else Serial.println(F("# RTC tidak menjawab"));
}

static void jalankan(char *baris) {
  char *t[8];
  int n = pecah(baris, t, 8);
  if (n == 0) return;
  for (char *c = t[0]; *c; c++) *c = tolower(*c);
  String pesan;
  const char *c = t[0];

  if (!strcmp(c, "bantuan") || !strcmp(c, "help") || !strcmp(c, "?")) bantuan();
  else if (!strcmp(c, "status")) cliCetakStatus();
  else if (!strcmp(c, "mulai") || !strcmp(c, "start")) jawab(fsmPerintahMulai("CLI", pesan), pesan);
  else if (!strcmp(c, "berhenti") || !strcmp(c, "stop") || !strcmp(c, "x")) {
    sUjiSuction = false;
    penggerakBerhenti(false);
    fsmPerintahBerhenti("perintah CLI");
    Serial.println(F("# berhenti"));
  }
  else if (!strcmp(c, "reset")) jawab(fsmPerintahReset(pesan), pesan);
  else if (!strcmp(c, "sensor")) {
    jarakCetak(Serial); imuCetak(Serial); amanCetak(Serial);
    long a, b; penggerakTick(a, b);
    Serial.printf("# encoder tick 1x: kiri %ld kanan %ld | VBAT %.2f V\n", a, b, dayaVbat());
  }
  else if (!strcmp(c, "i2c")) i2cPindai(Serial);
  else if (!strcmp(c, "log")) {
    sLog = n >= 2 ? atoi(t[1]) != 0 : !sLog;
    if (n >= 3) sLogPeriode = constrain(atol(t[2]), 50L, 5000L);
    if (sLog) Serial.println(F("t_ms,state,sub,v,w,x,y,th_deg,us_depan,us_kiri,us_kanan,tof_ki,tof_ka,tebing_kika,vbat,kotor,skor"));
  }
  else if (!strcmp(c, "param")) paramCetak(Serial);
  else if (!strcmp(c, "set") && n >= 3) jawab(paramSet(t[1], atof(t[2]), pesan), pesan);
  else if (!strcmp(c, "simpan")) Serial.println(paramSimpan() ? F("# parameter disimpan ke NVS \"robot\"") : F("# GAGAL simpan"));
  else if (!strcmp(c, "bawaan")) { paramBawaan(); Serial.println(F("# parameter kembali ke bawaan (belum disimpan)")); }
  else if (!strcmp(c, "muat")) Serial.println(paramMuat() ? F("# parameter dimuat dari NVS") : F("# tidak ada parameter tersimpan"));
  else if (!strcmp(c, "uji")) perintahUji(t, n);
  else if (!strcmp(c, "kamera")) perintahKamera(t, n);
  else if (!strcmp(c, "waktu")) perintahWaktu(t, n);
  else if (!strcmp(c, "jadwal")) {
    int h, m;
    if (n >= 2 && !strcmp(t[1], "off")) { P.jadwalAktif = 0; Serial.println(F("# jadwal mati (ketik 'simpan')")); }
    else if (n >= 2 && sscanf(t[1], "%d:%d", &h, &m) == 2 && h >= 0 && h < 24 && m >= 0 && m < 60) {
      P.jadwalAktif = 1; P.jadwalJam = h; P.jadwalMenit = m;
      Serial.printf("# jadwal harian %02d:%02d AKTIF (ketik 'simpan' agar tetap setelah reboot)\n", h, m);
    } else Serial.println(F("# format: jadwal 08:30 | jadwal off"));
  }
  else if (!strcmp(c, "gyro") && n >= 2 && !strcmp(t[1], "kalib")) {
    if (!penggerakDiam()) Serial.println(F("# robot harus diam"));
    else { imuMulaiKalibBias(100); Serial.println(F("# estimasi bias gyro z 2 s, jangan sentuh robot")); }
  }
  else if (!strcmp(c, "odo") && n >= 2 && !strcmp(t[1], "reset")) { penggerakResetOdo(); Serial.println(F("# pose = (0, 0, 0)")); }
  else if (!strcmp(c, "versi")) Serial.printf("# %s v%s, protokol kamera v%d, VL53L0X %s\n", FW_NAMA, FW_VERSI, PK_VERSI,
                                              tofPustakaAda() ? "pustaka Pololu" : "STUB (pustaka tidak terpasang)");
  else Serial.printf("# perintah '%s' tidak dikenal, ketik 'bantuan'\n", c);
}

void cliMulai() { sLen = 0; }

void cliPoll(uint32_t now) {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (sLen) { sBuf[sLen] = 0; jalankan(sBuf); sLen = 0; }
    } else if (sLen < sizeof(sBuf) - 1) {
      sBuf[sLen++] = ch;
    }
  }
  if (sUjiSuction && ((int32_t)(now - sSuctionSampai) >= 0 || fsmState() != ST_UJI)) {
    sUjiSuction = false;
    suctionSet(false);
    Serial.println(F("# suction uji mati"));
  }
}
