// param.cpp - tabel parameter, nilai bawaan, simpan/muat NVS
#include "param.h"
#include "config.h"
#include <Preferences.h>

Param P;

struct EntriParam {
  const char *nama;
  float *ptr;
  float minV, maksV;
  const char *ket;
};

static const EntriParam TABEL[] = {
  {"kp",          &P.kp,             0.0f, 20.0f, "gain P PID roda (per sampel 50 ms)"},
  {"ki",          &P.ki,             0.0f, 20.0f, "gain I PID roda"},
  {"kd",          &P.kd,             0.0f, 10.0f, "gain D PID roda"},
  {"amaks",       &P.aMaks,          0.05f, 2.0f, "m/s^2 akselerasi normal"},
  {"arem",        &P.aRem,           0.5f, 2.8f,  "m/s^2 rem darurat"},
  {"vbersih",     &P.vBersih,        0.05f, V_MAKS, "m/s kecepatan lajur"},
  {"vkotor",      &P.vKotor,         0.03f, V_MAKS, "m/s saat kotor"},
  {"vdekat",      &P.vDekat,         0.03f, V_MAKS, "m/s halangan dekat"},
  {"vmundur",     &P.vMundur,        0.03f, 0.2f,  "m/s mundur"},
  {"wputar",      &P.wPutar,         0.3f, 3.0f,  "rad/s laju putar maks"},
  {"khead",       &P.kHeading,       0.2f, 10.0f, "1/s gain heading"},
  {"lajur",       &P.lebarLajur,     0.05f, 0.5f, "m geser antar lajur"},
  {"lajurmaks",   &P.lajurMaks,      0.5f, 30.0f, "m panjang lajur maks"},
  {"durasimaks",  &P.durasiMaksMnt,  1.0f, 240.0f, "menit lama membersihkan"},
  {"jarakkamera", &P.jarakKamera,    0.0f, 1.0f,  "m titik pandang kamera ke nozzle"},
  {"ddepan",      &P.dDepanBelok,    5.0f, 100.0f, "cm US depan akhir lajur"},
  {"dlambat",     &P.dDepanLambat,   5.0f, 150.0f, "cm US depan perlambat"},
  {"ddarurat",    &P.dDepanDarurat,  3.0f, 50.0f, "cm US depan hindar"},
  {"dsamping",    &P.dSamping,       2.0f, 50.0f, "cm US samping menjauh"},
  {"dtof",        &P.dTofBelok,      30.0f, 1000.0f, "mm ToF serong akhir lajur"},
  {"dtofdarurat", &P.dTofDarurat,    20.0f, 500.0f, "mm ToF serong hindar"},
  {"mundur",      &P.mundurTebing,   0.03f, 0.2f, "m mundur setelah tebing"},
  {"modekotor",   &P.modeKotor,      0, 2,        "0 abaikan, 1 lambat, 2 lambat+ulang"},
  {"kotorkalib",  &P.kotorPerluKalib,0, 1,        "1 = abaikan kamera belum kalibrasi"},
  {"suction",     &P.suction,        0, 1,        "1 = suction nyala saat bersih"},
  {"tilt",        &P.tiltMaks,       5.0f, 60.0f, "derajat batas kemiringan"},
  {"alpha",       &P.alphaGyro,      0.0f, 1.0f,  "bobot gyro fusi heading"},
  {"vkoreksi",    &P.vbatKoreksi,    0.8f, 1.2f,  "faktor koreksi VBAT"},
  {"vwarn",       &P.vbatPeringatan, 12.0f, 16.8f, "V peringatan baterai"},
  {"vstop",       &P.vbatStop,       12.0f, 16.0f, "V berhenti (baterai habis)"},
  {"vpulih",      &P.vbatPulih,      13.0f, 16.8f, "V boleh mulai lagi"},
  {"jadwal",      &P.jadwalAktif,    0, 1,        "1 = mulai otomatis sesuai jadwal RTC"},
  {"jadwaljam",   &P.jadwalJam,      0, 23,       "jam jadwal"},
  {"jadwalmenit", &P.jadwalMenit,    0, 59,       "menit jadwal"},
};
static const size_t N_TABEL = sizeof(TABEL) / sizeof(TABEL[0]);

void paramBawaan() {
  P.kp = BAWAAN_KP; P.ki = BAWAAN_KI; P.kd = BAWAAN_KD;
  P.aMaks = BAWAAN_A_MAKS; P.aRem = BAWAAN_A_REM;
  P.vBersih = BAWAAN_V_BERSIH; P.vKotor = BAWAAN_V_KOTOR; P.vDekat = BAWAAN_V_DEKAT;
  P.vMundur = BAWAAN_V_MUNDUR; P.wPutar = BAWAAN_W_PUTAR; P.kHeading = BAWAAN_K_HEADING;
  P.lebarLajur = BAWAAN_LEBAR_LAJUR; P.lajurMaks = BAWAAN_LAJUR_MAKS;
  P.durasiMaksMnt = BAWAAN_DURASI_MAKS_MNT; P.jarakKamera = BAWAAN_JARAK_KAMERA;
  P.dDepanBelok = BAWAAN_D_DEPAN_BELOK; P.dDepanLambat = BAWAAN_D_DEPAN_LAMBAT;
  P.dDepanDarurat = BAWAAN_D_DEPAN_DARURAT; P.dSamping = BAWAAN_D_SAMPING;
  P.dTofBelok = BAWAAN_D_TOF_BELOK; P.dTofDarurat = BAWAAN_D_TOF_DARURAT;
  P.mundurTebing = BAWAAN_MUNDUR_TEBING;
  P.modeKotor = 1; P.kotorPerluKalib = 1; P.suction = 1;
  P.tiltMaks = BAWAAN_TILT_MAKS; P.alphaGyro = BAWAAN_ALPHA_GYRO;
  P.vbatKoreksi = BAWAAN_VBAT_KOREKSI; P.vbatPeringatan = BAWAAN_VBAT_PERINGATAN;
  P.vbatStop = BAWAAN_VBAT_STOP; P.vbatPulih = BAWAAN_VBAT_PULIH;
  P.jadwalAktif = 0; P.jadwalJam = 8; P.jadwalMenit = 0;
}

// Disimpan sebagai blob satu struct + versi; bila ukuran berubah (firmware baru), bawaan dipakai.
static const uint16_t PARAM_VERSI = 1;

bool paramMuat() {
  Preferences pref;
  if (!pref.begin("robot", true)) return false;
  bool ok = false;
  if (pref.getUShort("ver", 0) == PARAM_VERSI && pref.getBytesLength("p") == sizeof(Param)) {
    Param tmp;
    if (pref.getBytes("p", &tmp, sizeof(Param)) == sizeof(Param)) { P = tmp; ok = true; }
  }
  pref.end();
  return ok;
}

bool paramSimpan() {
  Preferences pref;
  if (!pref.begin("robot", false)) return false;
  bool ok = pref.putBytes("p", &P, sizeof(Param)) == sizeof(Param);
  pref.putUShort("ver", PARAM_VERSI);
  pref.end();
  return ok;
}

bool paramSet(const char *nama, float nilai, String &pesan) {
  for (size_t i = 0; i < N_TABEL; i++) {
    if (strcasecmp(nama, TABEL[i].nama) != 0) continue;
    if (nilai < TABEL[i].minV || nilai > TABEL[i].maksV) {
      pesan = String("nilai di luar batas ") + String(TABEL[i].minV, 3) + " .. " + String(TABEL[i].maksV, 3);
      return false;
    }
    *TABEL[i].ptr = nilai;
    pesan = String(TABEL[i].nama) + " = " + String(nilai, 4) + " (belum disimpan; ketik 'simpan')";
    return true;
  }
  pesan = "parameter tidak dikenal (ketik 'param')";
  return false;
}

void paramCetak(Stream &s) {
  s.println(F("# Parameter (set <nama> <nilai>, simpan, bawaan):"));
  for (size_t i = 0; i < N_TABEL; i++) {
    s.printf("  %-12s = %9.4f   [%g .. %g] %s\n", TABEL[i].nama, *TABEL[i].ptr,
             TABEL[i].minV, TABEL[i].maksV, TABEL[i].ket);
  }
}
