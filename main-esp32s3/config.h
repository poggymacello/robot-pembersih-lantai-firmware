// =====================================================================
//  config.h - Konfigurasi firmware utama (ESP32-S3 DevKit N16R8, PCB rev 004)
//
//  SEMUA nilai yang harus diukur / dikonfirmasi di perangkat keras ditandai [UKUR].
//  Nilai yang bisa diubah saat berjalan (perintah CLI "set") ada di param.cpp;
//  nilai di sini adalah konstanta kompilasi dan nilai bawaan parameter.
//  Peta pin: SPEK_BERSAMA bagian 3 dan README PCB Utama bagian 3.
// =====================================================================
#pragma once
#include <Arduino.h>

#define FW_NAMA  "main-esp32s3"
#define FW_VERSI "0.1.0"

#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
#error "Atur Tools > USB CDC On Boot = Disabled (GPIO19/20 dipakai LED G/B, upload lewat port COM/CH343)."
#endif

// =====================================================================
//  1. PETA PIN (tetap, sesuai PCB rev 004)
// =====================================================================
static constexpr uint8_t PIN_AIN1 = 15, PIN_AIN2 = 16, PIN_PWMA = 17;   // motor KIRI
static constexpr uint8_t PIN_BIN1 = 18, PIN_BIN2 = 8,  PIN_PWMB = 9;    // motor KANAN (STBY diikat 3V3)
static constexpr uint8_t PIN_ENC_KI_A = 4, PIN_ENC_KI_B = 5;            // encoder kiri C1, C2
static constexpr uint8_t PIN_ENC_KA_A = 6, PIN_ENC_KA_B = 7;            // encoder kanan C1, C2
static constexpr uint8_t PIN_SUCTION = 10;                              // gate MOSFET kipas turbin (ON/OFF saja)
static constexpr uint8_t PIN_VBAT = 1;                                  // ADC1, pembagi 100k/22k lewat JP1
static constexpr uint8_t PIN_ECHO_US1 = 11, PIN_ECHO_US4 = 14, PIN_ECHO_US5 = 21; // ECHO lewat pembagi 1k/2k
static constexpr uint8_t PIN_XSHUT_KI = 12, PIN_XSHUT_KA = 13;          // VL53L0X serong kiri / kanan
static constexpr uint8_t PIN_TCRT_KI = 41, PIN_TCRT_KA = 42;            // sensor tebing (digital)
static constexpr uint8_t PIN_SDA = 47, PIN_SCL = 48;
static constexpr uint8_t PIN_CAM_TX = 39, PIN_CAM_RX = 40;              // UART1 ke kamera (TX->U0R, RX<-U0T)
static constexpr uint8_t PIN_LED_R = 38, PIN_LED_G = 19, PIN_LED_B = 20;
static constexpr uint8_t PIN_BUZZER = 2;                                // lewat NPN, HIGH = bunyi
// GPIO 35, 36, 37 = PSRAM octal: JANGAN dipakai. GPIO3 cadangan (strapping).

// =====================================================================
//  2. ALAMAT I2C
// =====================================================================
static constexpr uint32_t I2C_HZ = 100000;          // PCF8574 maks 100 kHz
// [UKUR] PCF8574P (DIP, non-A, A0..A2 ke GND) = 0x20. Bila IC yang terpasang PCF8574AP: 0x38.
static constexpr uint8_t ADDR_PCF = 0x20;
static constexpr uint8_t ADDR_TOF_BAWAAN = 0x29;
static constexpr uint8_t ADDR_TOF_KI = 0x30, ADDR_TOF_KA = 0x31;
static constexpr uint8_t ADDR_EEPROM = 0x57;
static constexpr uint8_t ADDR_RTC = 0x68;
static constexpr uint8_t ADDR_MPU = 0x69;           // AD0 ke 3V3 di PCB

// Bit TRIG HC-SR04 di PCF8574
static constexpr uint8_t PCF_TRIG_US1 = 0, PCF_TRIG_US4 = 3, PCF_TRIG_US5 = 4;

// =====================================================================
//  3. PENGGERAK: TANDA, ENCODER, GEOMETRI   [UKUR]
// =====================================================================
// [UKUR] Konstanta tanda BELUM DIKONFIRMASI di PCB. Konvensi SAMA dengan Uji_PID_Kecepatan_Roda_v2
// dan Bringup_PCB menu 'm': SIGN_MOT = +1 bila IN1=HIGH membuat roda maju; SIGN_ENC = +1 bila
// hitungan naik saat roda maju. Nilai di bawah = nilai sementara Uji_PID v2 (uji breadboard 15 Okt).
// Tanda salah = umpan balik positif -> pengaman "lari" menghentikan motor (fault).
static constexpr int SIGN_ENC_KI = +1, SIGN_ENC_KA = -1;
static constexpr int SIGN_MOT_KI = +1, SIGN_MOT_KA = +1;

static constexpr float CPR_1X  = 517.0f;   // [UKUR] tick 1x per putaran roda (asumsi 11 PPR x 47). Putar roda 10x.
static constexpr float R_RODA  = 0.0335f;  // [UKUR] m, roda 67 mm (uji lurus 1-2 m untuk koreksi)
static constexpr float TRACK_L = 0.231f;   // [UKUR] m, jarak pusat roda (koreksi dengan uji putar / gyro)
static constexpr uint32_t TS_MS = 50;      // periode loop kecepatan (gain PID untuk 50 ms)
static constexpr float TS = TS_MS / 1000.0f;
static constexpr float M_PER_TICK = 2.0f * PI * R_RODA / CPR_1X;
static constexpr float TICK_PER_MPS = TS / M_PER_TICK;   // (tick/50 ms) per (m/s) ~ 122,8

static constexpr uint32_t PWM_FREQ_HZ = 20000;
static constexpr float U_MAX = 255.0f;
static constexpr float U_MIN_GERAK = 0.0f;  // [UKUR] kompensasi zona mati PWM (mis. 20) setelah diukur
static constexpr float V_MAKS = 0.30f;      // m/s batas mutlak (B200)

// Pengaman stall / tanda salah (sama dengan Uji_PID v2)
static constexpr float PWM_JENUH = 240.0f;
static constexpr uint32_t T_JENUH_MS = 400;
static constexpr float V_DIAM_TICK = 1.0f;   // tick 1x per 50 ms (~8 mm/s) dianggap diam

// Bawaan parameter PID / profil (bisa diubah dengan "set")
static constexpr float BAWAAN_KP = 0.75f, BAWAAN_KI = 2.25f, BAWAAN_KD = 0.25f;  // per sampel, Ts 50 ms
static constexpr float BAWAAN_A_MAKS = 0.5f;   // m/s^2 profil trapesium normal
static constexpr float BAWAAN_A_REM = 2.0f;    // m/s^2 rem darurat (batas tip-over ~2,84 m/s^2)

// =====================================================================
//  4. NAVIGASI (bawaan parameter)
// =====================================================================
static constexpr float BAWAAN_V_BERSIH = 0.20f;     // m/s kecepatan jalur lurus
static constexpr float BAWAAN_V_KOTOR = 0.10f;      // m/s saat kamera melihat kotor
static constexpr float BAWAAN_V_DEKAT = 0.12f;      // m/s saat halangan depan mulai dekat
static constexpr float BAWAAN_V_MUNDUR = 0.10f;     // m/s mundur
static constexpr float BAWAAN_W_PUTAR = 1.5f;       // rad/s batas laju putar di tempat
static constexpr float BAWAAN_K_HEADING = 2.5f;     // 1/s gain P heading (outer loop)
static constexpr float BAWAAN_LEBAR_LAJUR = 0.22f;  // m geser antar lajur (lebar sapuan 250 mm, overlap ~3 cm)
static constexpr float BAWAAN_LAJUR_MAKS = 6.0f;    // m panjang lajur maksimum sebelum belok paksa
static constexpr float BAWAAN_DURASI_MAKS_MNT = 45; // menit lama membersihkan maksimum
static constexpr float BAWAAN_JARAK_KAMERA = 0.30f; // [UKUR] m dari titik pandang kamera ke nozzle suction

// Ambang jarak [UKUR] (sesuaikan dengan posisi sensor di bumper)
static constexpr float BAWAAN_D_DEPAN_BELOK = 22.0f;   // cm US depan -> akhir lajur
static constexpr float BAWAAN_D_DEPAN_LAMBAT = 45.0f;  // cm US depan -> perlambat
static constexpr float BAWAAN_D_DEPAN_DARURAT = 10.0f; // cm US depan -> hindar (mundur dulu)
static constexpr float BAWAAN_D_SAMPING = 8.0f;        // cm US kiri/kanan -> menjauh dari dinding
static constexpr float BAWAAN_D_TOF_BELOK = 150.0f;    // mm ToF serong -> akhir lajur
static constexpr float BAWAAN_D_TOF_DARURAT = 70.0f;   // mm ToF serong -> hindar
static constexpr float BAWAAN_MUNDUR_TEBING = 0.10f;   // m mundur setelah tebing
static constexpr float MUNDUR_TEBING_MAKS = 0.25f;     // m bila tebing masih terbaca -> GALAT
static constexpr float MUNDUR_HINDAR = 0.06f;          // m mundur saat hindar

// =====================================================================
//  5. SENSOR JARAK
// =====================================================================
static constexpr float US_JAUH_CM = 400.0f;          // nilai bila tidak ada gema (bebas)
static constexpr uint32_t US_TIMEOUT_US = 25000;     // ~4,3 m pulang-pergi
static constexpr uint32_t US_JEDA_MS = 10;           // jeda antar sensor (gema sisa hilang)
static constexpr uint32_t US_ECHO_MACET_MS = 60;     // ECHO tetap HIGH selama ini -> macet
static constexpr uint8_t  US_GAGAL_MAKS = 15;        // berturut-turut tanpa respons -> sensor dianggap rusak
static constexpr uint32_t US_DATA_BASI_MS = 400;     // data lebih tua dari ini dianggap tidak valid
static constexpr int      TOF_JAUH_MM = 2000;        // >= ini dianggap bebas
static constexpr uint32_t TOF_PERIODE_MS = 40;
// VL53L0X: 0 = STUB (alamat diatur, jarak tidak diukur). 1 = pakai pustaka Pololu "VL53L0X"
// (pasang dulu lewat Library Manager). Tidak bisa dideteksi otomatis: penemu pustaka Arduino
// tidak mengikuti __has_include, jadi pilihan ini harus diubah manual.
#define PAKAI_PUSTAKA_VL53L0X 0

// =====================================================================
//  6. SENSOR TEBING TCRT5000   [UKUR]
// =====================================================================
// Modul umum: LOW = ada pantulan (lantai), HIGH = tidak ada pantulan (tebing). Cek dengan Bringup menu 'l'.
static constexpr int TCRT_LEVEL_TEBING = HIGH;
static constexpr uint8_t TCRT_DEBOUNCE = 2;          // sampel 10 ms berturut-turut

// =====================================================================
//  7. IMU MPU9250   [UKUR]
// =====================================================================
// Sumbu robot: +X depan, +Y kiri, +Z atas -> gyro z positif = putar kiri (CCW).
// [UKUR] Putar robot ke kiri dengan tangan: heading ("status") harus BERTAMBAH. Bila berkurang -> -1.
static constexpr int SIGN_GYRO_Z = +1;
static constexpr float BAWAAN_TILT_MAKS = 20.0f;      // derajat, lebih dari ini -> robot terangkat/terguling
static constexpr uint32_t TILT_TAHAN_MS = 300;
static constexpr float BAWAAN_ALPHA_GYRO = 0.98f;     // bobot gyro pada fusi heading (sisa = encoder)
static constexpr uint32_t IMU_PERIODE_MS = 20;        // 50 Hz

// =====================================================================
//  8. BATERAI 4S Li-ion   [UKUR]
// =====================================================================
static constexpr float VBAT_RASIO = 122.0f / 22.0f;   // (100k + 22k) / 22k
static constexpr float BAWAAN_VBAT_KOREKSI = 1.00f;   // [UKUR] bandingkan multimeter di TP1, set "vkoreksi"
static constexpr float BAWAAN_VBAT_PERINGATAN = 14.4f;// V (3,6 V/sel) beban penuh -> peringatan
static constexpr float BAWAAN_VBAT_STOP = 13.8f;      // V (3,45 V/sel) beban penuh -> berhenti
static constexpr float BAWAAN_VBAT_PULIH = 15.0f;     // V tanpa beban -> boleh mulai lagi (sudah diisi)
static constexpr float VBAT_HISTERESIS = 0.3f;        // V untuk lepas peringatan
static constexpr uint32_t VBAT_TAHAN_PERINGATAN_MS = 5000;
static constexpr uint32_t VBAT_TAHAN_STOP_MS = 3000;
static constexpr uint32_t VBAT_TAHAN_PULIH_MS = 10000;
static constexpr float VBAT_ABSEN_V = 6.0f;           // di bawah ini: sense tidak tersambung (uji di meja lewat USB)
static constexpr uint32_t VBAT_PERIODE_MS = 250;

// =====================================================================
//  9. SUCTION (kipas turbin brushless + driver internal, 15 V nominal, dari rel 12V-A)
// =====================================================================
// ON/OFF saja lewat MOSFET GPIO10 - TIDAK ADA PWM.
static constexpr uint32_t SUCTION_SPINUP_MS = 1500;   // tunggu putaran stabil sebelum mulai jalan

// =====================================================================
// 10. INDIKATOR
// =====================================================================
static constexpr bool LED_AKTIF_HIGH = true;          // [UKUR] katoda bersama = HIGH nyala

// =====================================================================
// 11. KAMERA
// =====================================================================
static constexpr uint32_t KAMERA_ABSEN_MS = 3000;     // tanpa bingkai valid selama ini -> kamera dianggap tidak ada
static constexpr uint16_t KAMERA_PERIODE_MS = 150;    // periode hasil yang diminta saat MULAI
