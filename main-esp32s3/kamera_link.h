// kamera_link.h - sisi ESP32-S3 dari protokol UART kamera (UART1, TX=GPIO39, RX=GPIO40, 115200 8N1)
#pragma once
#include <Arduino.h>

struct InfoKamera {
  // dari HEARTBEAT / STATUS
  uint8_t flags;           // PKF_*
  float fps;
  uint16_t sensorPid;      // 0x26 OV2640, 0x3660 OV3660, ...
  uint8_t errFrame;
  uint16_t uptimeS;
  uint8_t floorMean, floorStd, darkThr, progresKalib, papan;
  uint16_t normalDark, dirtyLimit, periodeMs;
  // hasil KOTOR terakhir
  uint8_t skor, kflags, region, skorKol[3], mean, nomorFrame;
  uint16_t dark, limit;
  uint32_t tKotorMs;       // millis() saat hasil KOTOR terakhir diterima
  uint32_t tRxMs;          // millis() saat bingkai valid terakhir diterima
  uint32_t nBingkai, nCrcSalah;
  uint8_t ackTipe, ackParam, ackStatus;
  uint8_t galatKode, galatDetail;
};

void kameraMulai();
void kameraPoll(uint32_t nowMs);          // baca UART, urus pemeliharaan (PING/MULAI ulang)
bool kameraAda();                         // bingkai valid dalam KAMERA_ABSEN_MS
void kameraInginDeteksi(bool aktif);      // FSM: true saat membersihkan (dikirim ulang bila kamera reboot)
bool kameraKotor();                       // keputusan kotor terbaru (false bila kamera absen / hasil basi)
const InfoKamera &kameraInfo();

// Perintah langsung (CLI)
void kameraKirimPing();
void kameraKirimMulai(uint16_t periodeMs);
void kameraKirimBerhenti();
void kameraKirimMintaStatus();
void kameraKirimSetParam(uint8_t id, int16_t nilai);
void kameraKirimKalibrasi();
void kameraKirimHapusKalib();
void kameraCetak(Stream &s);
void kameraSetEcho(bool aktif);           // cetak tiap hasil KOTOR ke Serial
