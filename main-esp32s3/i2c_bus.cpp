// i2c_bus.cpp - Wire dasar + PCF8574 (pola sama dengan Bringup_PCB)
#include "i2c_bus.h"
#include "config.h"
#include <Wire.h>

static bool sPcfAda = false;
static uint8_t sPcfShadow = 0x00;
static uint32_t sPcfGagal = 0;

void i2cMulai() {
  Wire.begin((int)PIN_SDA, (int)PIN_SCL, (uint32_t)I2C_HZ);
  Wire.setTimeOut(20);   // ms, jangan menggantung loop bila bus macet
}

bool i2cAda(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cTulisReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cBacaReg(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;   // STOP, aman untuk semua perangkat di bus
  if (Wire.requestFrom(addr, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

int i2cBaca1(uint8_t addr, uint8_t reg) {
  uint8_t v;
  return i2cBacaReg(addr, reg, &v, 1) ? v : -1;
}

void i2cPindai(Stream &s) {
  static const uint8_t HARAP[] = {ADDR_PCF, ADDR_TOF_KI, ADDR_TOF_KA, ADDR_EEPROM, ADDR_RTC, ADDR_MPU};
  s.print(F("# I2C ditemukan:"));
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cAda(a)) { s.printf(" 0x%02X", a); n++; }
  }
  s.println(n ? "" : " TIDAK ADA");
  s.print(F("# Diharapkan  :"));
  for (uint8_t a : HARAP) s.printf(" 0x%02X%s", a, i2cAda(a) ? "" : "(HILANG)");
  s.println();
  if (i2cAda(ADDR_TOF_BAWAAN)) s.println(F("# PERHATIAN: ada perangkat di 0x29 (VL53L0X belum diberi alamat / XSHUT tidak bekerja)."));
}

// ---------------- PCF8574 ----------------
bool pcfTulis(uint8_t val) {
  Wire.beginTransmission(ADDR_PCF);
  Wire.write(val);
  bool ok = Wire.endTransmission() == 0;
  if (ok) sPcfShadow = val; else sPcfGagal++;
  return ok;
}

bool pcfSetBit(uint8_t bit, bool tinggi) {
  uint8_t v = tinggi ? (sPcfShadow | (1 << bit)) : (sPcfShadow & ~(1 << bit));
  return pcfTulis(v);
}

bool pcfMulai() {
  sPcfAda = i2cAda(ADDR_PCF);
  if (sPcfAda) sPcfAda = pcfTulis(0x00);   // PCF8574 menyala dengan semua pin HIGH -> TRIG LOW-kan
  return sPcfAda;
}

bool pcfAda() { return sPcfAda; }
uint32_t pcfJumlahGagal() { return sPcfGagal; }
