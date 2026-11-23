// =====================================================================
//  FIRMWARE UTAMA - Robot Pembersih Lantai Otonom
//  Board  : ESP32-S3 DevKit N16R8 (klon DevKitC-1) di PCB utama rev 004
//  Core   : arduino-esp32 3.x (diuji kompilasi 3.3.12)
//  Tools  : ESP32S3 Dev Module, PSRAM "OPI PSRAM", Flash 16MB, USB CDC On Boot "Disabled",
//           upload lewat port USB-C "COM" (CH343). Serial Monitor 115200, akhir baris Newline.
//
//  Struktur (satu folder sketch, semua modul dikompilasi bersama):
//    config.h          pin, alamat I2C, konstanta yang WAJIB DIUKUR [UKUR]
//    param.*           parameter runtime (CLI set/simpan, NVS "robot")
//    i2c_bus.*         Wire + PCF8574
//    penggerak.*       motor, encoder 4x, PID 20 Hz, profil trapesium, odometri, pengaman stall
//    sensor_jarak.*    HC-SR04 x3 (non-blocking) + VL53L0X x2 (pustaka Pololu bila ada, selain itu STUB)
//    sensor_imu.*      MPU9250 gyro z + kemiringan, kalibrasi NVS "imu"
//    rtc_ds3231.*      waktu + jadwal
//    daya.*            baterai 4S + suction (ON/OFF)
//    indikator.*       LED RGB + buzzer
//    kamera_link.*     protokol UART kamera (protokol_kamera.h dipakai bersama sketch kamera)
//    keselamatan.*     supervisor keselamatan (menimpa FSM)
//    fsm.*             mesin keadaan + cakupan bolak-balik
//    cli.*             perintah serial
//
//  Penjadwal (millis, tanpa delay di loop):
//    tiap loop : CLI, UART kamera, HC-SR04, indikator
//    10 ms     : supervisor keselamatan (TCRT, kemiringan, batas gerak)
//    20 ms     : IMU (50 Hz)
//    40 ms     : VL53L0X (bila ada pustaka)
//    50 ms     : inner loop kecepatan roda (Ts = 50 ms, gain hasil simulasi)
//    100 ms    : FSM / outer loop (10 Hz)
//    250 ms    : baterai
//    1000 ms   : jadwal RTC
// =====================================================================

#include "config.h"
#include "param.h"
#include "i2c_bus.h"
#include "penggerak.h"
#include "sensor_jarak.h"
#include "sensor_imu.h"
#include "rtc_ds3231.h"
#include "daya.h"
#include "indikator.h"
#include "kamera_link.h"
#include "keselamatan.h"
#include "fsm.h"
#include "cli.h"

// Keadaan aman PALING AWAL: semua aktuator mati, kedua VL53L0X dimatikan (XSHUT LOW).
static void keadaanAmanAwal() {
  const uint8_t keluaranLow[] = {PIN_AIN1, PIN_AIN2, PIN_PWMA, PIN_BIN1, PIN_BIN2, PIN_PWMB,
                                 PIN_SUCTION, PIN_XSHUT_KI, PIN_XSHUT_KA, PIN_BUZZER,
                                 PIN_LED_R, PIN_LED_G, PIN_LED_B};
  for (uint8_t p : keluaranLow) { pinMode(p, OUTPUT); digitalWrite(p, LOW); }
}

static void selftest() {
  Serial.println(F("# ---------- SELFTEST ----------"));
  Serial.printf("# chip %s rev %d, flash %lu MB, PSRAM %lu KB\n", ESP.getChipModel(), ESP.getChipRevision(),
                (unsigned long)(ESP.getFlashChipSize() >> 20), (unsigned long)(ESP.getPsramSize() >> 10));

  bool pcf = pcfMulai();
  Serial.printf("# PCF8574 0x%02X: %s\n", ADDR_PCF, pcf ? "OK" : "TIDAK ADA -> HC-SR04 tidak bisa dipicu, 'mulai' ditolak");

  String lap;
  bool tof = tofMulai(lap);
  Serial.printf("# VL53L0X: %s%s\n", lap.c_str(), tof ? "" : "(cek kabel/XSHUT, lihat README PCB 5.1)");
  if (!tofPustakaAda())
    Serial.println(F("# VL53L0X: PAKAI_PUSTAKA_VL53L0X = 0 -> jarak serong STUB (tidak dipakai). Pasang pustaka Pololu \"VL53L0X\" lalu set 1 di config.h."));

  bool imu = imuMulai();
  Serial.printf("# MPU9250 0x%02X: %s WHO_AM_I 0x%02X, kalibrasi NVS %s\n", ADDR_MPU, imu ? "OK" : "GAGAL",
                imuWhoAmI(), imuKalibDariNvs() ? "dimuat" : "TIDAK ADA (jalankan Kalibrasi_IMU, bias diestimasi saat boot)");

  if (rtcMulai()) {
    WaktuRtc w;
    rtcBaca(w);
    Serial.print(F("# DS3231: OK, "));
    rtcCetak(Serial, w);
    Serial.println();
  } else Serial.println(F("# DS3231: TIDAK ADA (jadwal tidak bisa dipakai)"));

  i2cPindai(Serial);
  dayaUpdate(millis());
  Serial.printf("# VBAT %.2f V (%s)\n", dayaVbat(), dayaNamaStatus());
  Serial.printf("# tebing KI=%d KA=%d (1 = tidak ada lantai)\n",
                digitalRead(PIN_TCRT_KI) == TCRT_LEVEL_TEBING, digitalRead(PIN_TCRT_KA) == TCRT_LEVEL_TEBING);
  Serial.printf("# SIGN_MOT %+d/%+d SIGN_ENC %+d/%+d SIGN_GYRO_Z %+d (BELUM DIKONFIRMASI bila belum diuji)\n",
                SIGN_MOT_KI, SIGN_MOT_KA, SIGN_ENC_KI, SIGN_ENC_KA, SIGN_GYRO_Z);
  Serial.println(F("# Robot DIAM 2 s (estimasi bias gyro), menunggu heartbeat kamera..."));
}

void setup() {
  keadaanAmanAwal();
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n# ===== %s v%s - Robot Pembersih Lantai Otonom =====\n", FW_NAMA, FW_VERSI);

  paramBawaan();
  Serial.println(paramMuat() ? F("# parameter dimuat dari NVS \"robot\"") : F("# parameter bawaan (NVS kosong)"));

  indikatorMulai();
  ledSetPola(LED_BOOT);
  dayaMulai();
  penggerakMulai();
  i2cMulai();
  usMulai();
  amanMulai();
  kameraMulai();
  cliMulai();
  selftest();
  fsmMulai();
  enableLoopWDT();   // loop() macet > 5 s -> reset; saat reset semua aktuator kembali mati
  Serial.println(F("# Ketik 'bantuan' untuk daftar perintah."));
}

void loop() {
  static uint32_t t10 = 0, t20 = 0, t40 = 0, t50 = 0, t100 = 0, t250 = 0, t1000 = 0, tLog = 0;
  uint32_t now = millis();

  cliPoll(now);
  kameraPoll(now);
  usPoll();
  indikatorUpdate(now);

  if (now - t10 >= 10) { t10 = now; amanUpdate(now); }
  if (now - t20 >= IMU_PERIODE_MS) { t20 = now; imuUpdate(now); }
  if (now - t40 >= TOF_PERIODE_MS) { t40 = now; tofPoll(now); }
  if (now - t50 >= TS_MS) {
    // jaga periode tetap (gain PID dirancang untuk Ts 50 ms); bila tertinggal jauh, sinkron ulang
    t50 = (now - t50 > 2 * TS_MS) ? now : t50 + TS_MS;
    penggerakUpdate(now);
  }
  if (now - t100 >= 100) {
    t100 = now;
    fsmUpdate(now);
    amanUpdate(now);   // terapkan lagi SETELAH FSM: keselamatan selalu menang
  }
  if (now - t250 >= VBAT_PERIODE_MS) { t250 = now; dayaUpdate(now); }
  if (now - t1000 >= 1000) { t1000 = now; fsmTiapDetik(now); }
  if (cliLogAktif() && now - tLog >= cliLogPeriode()) { tLog = now; cliCetakLog(); }
}
