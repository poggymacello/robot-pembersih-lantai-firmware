// =====================================================================
//  papan_kamera.h - pemilihan papan kamera dan peta pin
//
//  Papan yang ada: header berlabel gaya AI-Thinker ESP32-CAM
//    (3V3 IO16 IO0 GND VCC U0R U0T GND / 5V GND IO12 IO13 IO15 IO14 IO2 IO4), sensor OV3660.
//  Bisa ESP32 klasik (AI-Thinker) ATAU klon ESP32-S3-CAM. Pilih dengan PAPAN_KAMERA di bawah,
//  atau biarkan otomatis: target ESP32 -> AI-Thinker, target ESP32-S3 -> peta ESP32-S3-EYE.
//
//  !!! PETA PIN ESP32-S3 DI BAWAH ADALAH ASUMSI (ESP32-S3-EYE / Freenove ESP32-S3 WROOM CAM,
//  sama dengan CAMERA_MODEL_ESP32S3_EYE pada contoh CameraWebServer). Klon ESP32-S3-CAM berbentuk
//  AI-Thinker bisa memakai peta lain. WAJIB dicocokkan dengan skematik/penjual papan sebelum dipakai.
//  Gejala peta salah: "init kamera GAGAL 0x105/0x20004" di konsol. Bila beda, pakai PAPAN_KUSTOM.
// =====================================================================
#pragma once
#include <Arduino.h>
#include "protokol_kamera.h"

#define PAPAN_AITHINKER 1
#define PAPAN_S3_EYE    2
#define PAPAN_KUSTOM    3

// ---- PILIH PAPAN (hapus komentar salah satu; tanpa pilihan = otomatis dari target) ----
// #define PAPAN_KAMERA PAPAN_AITHINKER
// #define PAPAN_KAMERA PAPAN_S3_EYE
// #define PAPAN_KAMERA PAPAN_KUSTOM

#ifndef PAPAN_KAMERA
  #if defined(CONFIG_IDF_TARGET_ESP32S3)
    #define PAPAN_KAMERA PAPAN_S3_EYE
  #else
    #define PAPAN_KAMERA PAPAN_AITHINKER
  #endif
#endif

#if PAPAN_KAMERA == PAPAN_AITHINKER
  #if !defined(CONFIG_IDF_TARGET_ESP32)
    #error "PAPAN_AITHINKER butuh chip ESP32 klasik: Tools > Board = AI Thinker ESP32-CAM"
  #endif
  #define PAPAN_NAMA   "AI-Thinker ESP32-CAM"
  #define PAPAN_ID     PK_PAPAN_AITHINKER
  // Peta pin AI-Thinker ESP32-CAM (sudah baku)
  #define CAM_PIN_PWDN    32
  #define CAM_PIN_RESET   -1
  #define CAM_PIN_XCLK     0
  #define CAM_PIN_SIOD    26
  #define CAM_PIN_SIOC    27
  #define CAM_PIN_D7      35   // Y9
  #define CAM_PIN_D6      34   // Y8
  #define CAM_PIN_D5      39   // Y7
  #define CAM_PIN_D4      36   // Y6
  #define CAM_PIN_D3      21   // Y5
  #define CAM_PIN_D2      19   // Y4
  #define CAM_PIN_D1      18   // Y3
  #define CAM_PIN_D0       5   // Y2
  #define CAM_PIN_VSYNC   25
  #define CAM_PIN_HREF    23
  #define CAM_PIN_PCLK    22
  #define PIN_LED_FLASH    4   // LED putih terang (ON = HIGH). Dimatikan: silau di jendela.
  // UART ke ESP32-S3 utama = UART0 di header U0R (GPIO3) / U0T (GPIO1), sama dengan jalur upload/log boot
  #define LINK_PIN_RX      3
  #define LINK_PIN_TX      1

#elif PAPAN_KAMERA == PAPAN_S3_EYE
  #if !defined(CONFIG_IDF_TARGET_ESP32S3)
    #error "PAPAN_S3_EYE butuh chip ESP32-S3: Tools > Board = ESP32S3 Dev Module"
  #endif
  #define PAPAN_NAMA   "ESP32-S3-CAM (peta ESP32-S3-EYE, ASUMSI)"
  #define PAPAN_ID     PK_PAPAN_S3_EYE
  #define CAM_PIN_PWDN    -1
  #define CAM_PIN_RESET   -1
  #define CAM_PIN_XCLK    15
  #define CAM_PIN_SIOD     4
  #define CAM_PIN_SIOC     5
  #define CAM_PIN_D7      16   // Y9
  #define CAM_PIN_D6      17   // Y8
  #define CAM_PIN_D5      18   // Y7
  #define CAM_PIN_D4      12   // Y6
  #define CAM_PIN_D3      10   // Y5
  #define CAM_PIN_D2       8   // Y4
  #define CAM_PIN_D1       9   // Y3
  #define CAM_PIN_D0      11   // Y2
  #define CAM_PIN_VSYNC    6
  #define CAM_PIN_HREF     7
  #define CAM_PIN_PCLK    13
  #define PIN_LED_FLASH   -1   // [CEK] isi GPIO LED flash bila papan punya
  // [CEK] UART0 ESP32-S3: U0TXD = GPIO43, U0RXD = GPIO44 (label U0T/U0R pada header)
  #define LINK_PIN_RX     44
  #define LINK_PIN_TX     43

#elif PAPAN_KAMERA == PAPAN_KUSTOM
  // Isi semua nilai dari skematik papan yang dipakai.
  #define PAPAN_NAMA   "kustom"
  #define PAPAN_ID     PK_PAPAN_KUSTOM
  #define CAM_PIN_PWDN    -1
  #define CAM_PIN_RESET   -1
  #define CAM_PIN_XCLK    -1
  #define CAM_PIN_SIOD    -1
  #define CAM_PIN_SIOC    -1
  #define CAM_PIN_D7      -1
  #define CAM_PIN_D6      -1
  #define CAM_PIN_D5      -1
  #define CAM_PIN_D4      -1
  #define CAM_PIN_D3      -1
  #define CAM_PIN_D2      -1
  #define CAM_PIN_D1      -1
  #define CAM_PIN_D0      -1
  #define CAM_PIN_VSYNC   -1
  #define CAM_PIN_HREF    -1
  #define CAM_PIN_PCLK    -1
  #define PIN_LED_FLASH   -1
  #define LINK_PIN_RX     -1
  #define LINK_PIN_TX     -1
  #error "PAPAN_KUSTOM: isi peta pin di papan_kamera.h lalu hapus baris #error ini"
#else
  #error "PAPAN_KAMERA tidak dikenal"
#endif
