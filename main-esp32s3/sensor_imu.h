// sensor_imu.h - MPU9250 (0x69): gyro z untuk heading, akselerometer untuk kemiringan.
// Kalibrasi bias dimuat dari NVS namespace "imu" (dibuat sketch Kalibrasi_IMU).
#pragma once
#include <Arduino.h>

bool imuMulai();                    // reset + konfigurasi + WHO_AM_I + muat NVS
void imuUpdate(uint32_t nowMs);     // panggil tiap IMU_PERIODE_MS
bool imuOk();                       // data terbaru valid
uint8_t imuWhoAmI();
bool imuKalibDariNvs();
float imuGyroZ();                   // rad/s, bias dibuang, tanda sumbu robot (CCW +)
float imuTiltDeg();                 // sudut terhadap vertikal (0 = datar)
float imuAmbilDeltaSudut();         // rad akumulasi sejak panggilan terakhir (lalu direset)
// Estimasi ulang bias gyro z selama robot diam (non-blocking), mis. saat BOOT dan START
void imuMulaiKalibBias(uint16_t nSampel);
bool imuKalibBiasSibuk();
bool imuKalibBiasBerhasil();
float imuBiasZdps();
void imuCetak(Stream &s);
