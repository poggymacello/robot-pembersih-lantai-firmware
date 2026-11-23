// cli.h - antarmuka perintah serial (Serial = UART0 lewat CH343, 115200, akhir baris NL atau CR)
#pragma once
#include <Arduino.h>

void cliMulai();
void cliPoll(uint32_t nowMs);        // baca karakter, jalankan perintah, urus batas waktu uji suction
bool cliLogAktif();
uint32_t cliLogPeriode();
void cliCetakLog();
void cliCetakStatus();
