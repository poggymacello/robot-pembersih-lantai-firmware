// daya.h - pemantau baterai 4S Li-ion (GPIO1) dan suction kipas turbin (GPIO10, ON/OFF)
#pragma once
#include <Arduino.h>

enum StatusBaterai : uint8_t { BAT_NORMAL = 0, BAT_PERINGATAN, BAT_KRITIS, BAT_ABSEN };

void dayaMulai();
void dayaUpdate(uint32_t nowMs);       // panggil tiap VBAT_PERIODE_MS
float dayaVbat();                      // V tersaring
float dayaVbatSesaat();
StatusBaterai dayaStatus();            // KRITIS terkunci sampai VBAT > vpulih selama 10 s
const char *dayaNamaStatus();

void suctionSet(bool nyala);
bool suctionNyala();
uint32_t suctionLamaNyalaMs();         // 0 bila mati
