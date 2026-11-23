// i2c_bus.h - fungsi bantu I2C (Wire) dan PCF8574. Hanya dipanggil dari loop() (satu thread).
#pragma once
#include <Arduino.h>

void i2cMulai();
bool i2cAda(uint8_t addr);
bool i2cTulisReg(uint8_t addr, uint8_t reg, uint8_t val);
bool i2cBacaReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n);
int  i2cBaca1(uint8_t addr, uint8_t reg);   // -1 bila gagal
void i2cPindai(Stream &s);

// PCF8574: semua pin keluaran; shadow byte disimpan agar bit lain tidak berubah.
bool pcfMulai();               // tulis 0x00 (semua TRIG LOW)
bool pcfAda();
bool pcfTulis(uint8_t val);
bool pcfSetBit(uint8_t bit, bool tinggi);
uint32_t pcfJumlahGagal();
