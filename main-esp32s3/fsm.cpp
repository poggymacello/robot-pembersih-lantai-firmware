// fsm.cpp - mesin keadaan, primitif gerak (manuver) dan algoritma cakupan
#include "fsm.h"
#include "config.h"
#include "param.h"
#include "penggerak.h"
#include "sensor_jarak.h"
#include "sensor_imu.h"
#include "rtc_ds3231.h"
#include "daya.h"
#include "indikator.h"
#include "kamera_link.h"
#include "keselamatan.h"
#include "i2c_bus.h"

// =====================================================================
//  Utilitas
// =====================================================================
static float bungkusPi(float a) {
  while (a > PI) a -= 2.0f * PI;
  while (a < -PI) a += 2.0f * PI;
  return a;
}
static const float DEG = PI / 180.0f;

// Jarak halangan ringkas (US cm dan ToF mm). -1/tidak valid dianggap "tidak tahu" = bebas
// kecuali sensor depan rusak (ditangani terpisah).
static float usCm(uint8_t id) { float d = usJarakCm(id); return d < 0 ? US_JAUH_CM : d; }
static float tofMinMm() {
  int a = tofJarakMm(TOF_KI), b = tofJarakMm(TOF_KA);
  float m = TOF_JAUH_MM;
  if (a >= 0 && a < m) m = a;
  if (b >= 0 && b < m) m = b;
  return m;
}
static bool depanTerhalang() { return usCm(US_DEPAN) < P.dDepanBelok || tofMinMm() < P.dTofBelok; }
static bool depanDarurat() { return usCm(US_DEPAN) < P.dDepanDarurat || tofMinMm() < P.dTofDarurat; }

// =====================================================================
//  Manuver: urutan langkah gerak sederhana dengan heading gyro
// =====================================================================
enum TipeLangkah : uint8_t { L_MAJU = 1, L_MUNDUR, L_PUTAR, L_TUNGGU };
enum HasilManuver : uint8_t { M_JALAN = 0, M_SELESAI, M_TERHALANG, M_TIMEOUT };

struct Langkah { TipeLangkah t; float nilai; };   // m, rad (relatif, + = kiri/CCW), ms
struct Manuver {
  Langkah l[4];
  uint8_t n = 0, i = 0;
  bool aktif = false, jeda = false;
  float s0 = 0, thTarget = 0, vMaju = 0;
  uint32_t t0 = 0, tBatas = 0;
};

static Manuver sMan;

static void langkahMulai(uint32_t now) {
  Pose p = penggerakPose();
  Langkah &L = sMan.l[sMan.i];
  sMan.s0 = p.s;
  sMan.t0 = now;
  sMan.jeda = false;
  switch (L.t) {
    case L_MAJU:
      sMan.thTarget = p.th;
      sMan.tBatas = (uint32_t)(1000.0f * L.nilai / fmaxf(sMan.vMaju, 0.03f) * 2.0f) + 2000;
      break;
    case L_MUNDUR:
      sMan.thTarget = p.th;
      sMan.tBatas = (uint32_t)(1000.0f * L.nilai / fmaxf(P.vMundur, 0.03f) * 2.0f) + 2000;
      break;
    case L_PUTAR:
      sMan.thTarget = p.th + L.nilai;
      sMan.tBatas = (uint32_t)(1000.0f * fabsf(L.nilai) / (0.4f * P.wPutar)) + 3000;
      break;
    case L_TUNGGU:
      sMan.tBatas = (uint32_t)L.nilai + 1000;
      break;
  }
}

static void manuverMulai(const Langkah *daftar, uint8_t n, float vMaju) {
  sMan.n = n > 4 ? 4 : n;
  for (uint8_t i = 0; i < sMan.n; i++) sMan.l[i] = daftar[i];
  sMan.i = 0;
  sMan.vMaju = vMaju;
  sMan.aktif = true;
  langkahMulai(millis());
}

static void manuverBatal() { sMan.aktif = false; penggerakBerhenti(false); }

// Kembalikan hasil; panggil tiap siklus FSM saat sMan.aktif.
static HasilManuver manuverUpdate(uint32_t now) {
  if (!sMan.aktif) return M_SELESAI;
  Pose p = penggerakPose();
  // Jeda antar langkah: tunggu robot berhenti (rampa) sebelum langkah berikutnya
  if (sMan.jeda) {
    penggerakSetKecepatan(0, 0);
    if (penggerakDiam() || now - sMan.t0 > 1000) {
      sMan.i++;
      if (sMan.i >= sMan.n) { sMan.aktif = false; return M_SELESAI; }
      langkahMulai(now);
    }
    return M_JALAN;
  }
  Langkah &L = sMan.l[sMan.i];
  if (now - sMan.t0 > sMan.tBatas) { manuverBatal(); return M_TIMEOUT; }
  bool selesai = false;
  switch (L.t) {
    case L_MAJU: {
      if (depanTerhalang()) { manuverBatal(); return M_TERHALANG; }
      float w = constrain(P.kHeading * bungkusPi(sMan.thTarget - p.th), -0.8f, 0.8f);
      penggerakSetKecepatan(sMan.vMaju, w);
      selesai = (p.s - sMan.s0) >= L.nilai;
      break;
    }
    case L_MUNDUR: {
      // Tanpa sensor belakang: jarak mundur selalu pendek. Heading dijaga (w = 0 bila tebing).
      float w = constrain(P.kHeading * bungkusPi(sMan.thTarget - p.th), -0.5f, 0.5f);
      penggerakSetKecepatan(-P.vMundur, w);
      selesai = (sMan.s0 - p.s) >= L.nilai;
      break;
    }
    case L_PUTAR: {
      float e = bungkusPi(sMan.thTarget - p.th);
      float w = constrain(P.kHeading * 1.5f * e, -P.wPutar, P.wPutar);
      if (fabsf(w) < 0.35f) w = (e >= 0 ? 0.35f : -0.35f);   // laju minimum (zona mati)
      if (fabsf(e) < 3.0f * DEG) { w = 0; selesai = true; }
      penggerakSetKecepatan(0, w);
      break;
    }
    case L_TUNGGU:
      penggerakSetKecepatan(0, 0);
      selesai = now - sMan.t0 >= (uint32_t)L.nilai;
      break;
  }
  if (selesai) { sMan.jeda = true; sMan.t0 = now; penggerakSetKecepatan(0, 0); }
  return M_JALAN;
}

// =====================================================================
//  State
// =====================================================================
enum SubBersih : uint8_t { SB_PERSIAPAN = 0, SB_LURUS, SB_BELOK_U, SB_ULANG_KOTOR };

static StateRobot sSt = ST_BOOT, sStPrev = ST_BOOT;
static SubBersih sSub = SB_PERSIAPAN;
static uint32_t sTMasuk = 0, sTMulaiBersih = 0;
static char sAlasanGalat[160] = "";
static bool sMasukBaru = true;

// Cakupan
static float sLajurHeading = 0;     // rad
static int8_t sSisi = +1;           // +1 = geser ke kiri, -1 = ke kanan
static float sSLajurMulai = 0;
static uint8_t sGeserGagal = 0;   // jumlah sisi yang sudah habis (geser lajur terhalang)
// Kotor
static float sLambatSampaiS = -1e9f, sKotorMulaiS = 0;
static bool sEpisodeKotor = false, sEpisodeDiulang = false;
static float sZonaUlangSampaiS = -1e9f;   // dalam zona lintasan ulang: jangan ulangi lagi (cegah putaran tanpa akhir)
// Hindar / tebing
static uint32_t sRiwayatHindar[4] = {0, 0, 0, 0};
static uint32_t sRiwayatTebing[5] = {0, 0, 0, 0, 0};
static bool sTebingKiAwal = false, sTebingKaAwal = false;
static float sTebingMundurTotal = 0;
static uint8_t sTebingFase = 0;
// Uji
static uint32_t sUjiSampai = 0;
static bool sUjiJalan = false;
// Jadwal
static uint16_t sJadwalTglTerakhir = 0;

static const char *NAMA_ST[] = {"BOOT", "IDLE", "BERSIH", "HINDAR", "TEBING", "BATERAI", "GALAT", "UJI"};
static const char *NAMA_SUB[] = {"persiapan", "lurus", "belok-U", "ulang-kotor"};

StateRobot fsmState() { return sSt; }
const char *fsmNamaState() { return NAMA_ST[sSt]; }
const char *fsmNamaSub() { return (sSt == ST_BERSIH) ? NAMA_SUB[sSub] : "-"; }
const char *fsmAlasanGalat() { return sAlasanGalat; }

static void pindah(StateRobot st) {
  if (st == sSt) return;
  Serial.printf("# STATE %s -> %s\n", NAMA_ST[sSt], NAMA_ST[st]);
  sStPrev = sSt;
  sSt = st;
  sTMasuk = millis();
  sMasukBaru = true;
  sMan.aktif = false;
}

// catat kejadian di riwayat bergulir, kembalikan true bila jumlah dalam jendela >= n
static bool catatRiwayat(uint32_t *r, uint8_t n, uint32_t jendelaMs) {
  uint32_t now = millis();
  for (uint8_t i = n - 1; i > 0; i--) r[i] = r[i - 1];
  r[0] = now;
  return r[n - 1] != 0 && now - r[n - 1] < jendelaMs;
}

static void semuaAman(bool darurat) {
  penggerakBerhenti(darurat);
  suctionSet(false);
  kameraInginDeteksi(false);
  usSetAktif(true);
}

void fsmGalat(const char *alasan) {
  strncpy(sAlasanGalat, alasan, sizeof(sAlasanGalat) - 1);
  sAlasanGalat[sizeof(sAlasanGalat) - 1] = 0;
  Serial.printf("# GALAT: %s\n", sAlasanGalat);
  pindah(ST_GALAT);
}

static bool stateBergerak() { return sSt == ST_BERSIH || sSt == ST_HINDAR || sSt == ST_TEBING || sSt == ST_UJI; }

// =====================================================================
//  Perintah
// =====================================================================
void fsmMulai() {
  sSt = ST_BOOT;
  sTMasuk = millis();
  sMasukBaru = true;
}

bool fsmPerintahMulai(const char *sumber, String &pesan) {
  if (sSt == ST_UJI) { pindah(ST_IDLE); amanAbaikanTebing(false); }
  if (sSt != ST_IDLE) { pesan = String("ditolak: state ") + NAMA_ST[sSt] + " (harus IDLE)"; return false; }
  if (dayaStatus() == BAT_KRITIS) { pesan = "ditolak: baterai kritis"; return false; }
  if (penggerakFault()) { pesan = "ditolak: fault roda, ketik 'reset'"; return false; }
  if (!pcfAda() || usRusak(US_DEPAN)) { pesan = "ditolak: HC-SR04 depan / PCF8574 tidak bekerja (cek 'sensor')"; return false; }
  if (amanTebing()) { pesan = "ditolak: sensor tebing aktif (robot tidak di lantai?)"; return false; }
  if (amanMiring()) { pesan = "ditolak: robot miring"; return false; }
  pesan = String("mulai membersihkan (sumber: ") + sumber + ")";
  if (!imuOk()) pesan += " PERINGATAN: IMU tidak OK, heading hanya dari encoder";
  if (!kameraAda()) pesan += " PERINGATAN: kamera tidak ada, tanpa deteksi kotor";
  if (!tofTersedia()) pesan += " PERINGATAN: ToF serong tidak aktif";
  pindah(ST_BERSIH);
  return true;
}

void fsmPerintahBerhenti(const char *alasan) {
  if (sSt == ST_GALAT || sSt == ST_BATERAI || sSt == ST_BOOT) { semuaAman(false); return; }
  Serial.printf("# berhenti: %s\n", alasan);
  semuaAman(false);
  amanAbaikanTebing(false);
  pindah(ST_IDLE);
}

bool fsmPerintahReset(String &pesan) {
  if (penggerakFault()) penggerakLepasFault();
  if (sSt != ST_GALAT && sSt != ST_UJI) { pesan = "tidak ada galat"; return true; }
  if (amanMiring()) { pesan = "ditolak: robot masih miring"; return false; }
  sAlasanGalat[0] = 0;
  buzzerDiam();
  semuaAman(false);
  amanAbaikanTebing(false);
  pindah(ST_IDLE);
  pesan = "galat dilepas, kembali IDLE";
  return true;
}

bool fsmUjiMasuk(String &pesan) {
  if (sSt == ST_UJI) return true;
  if (sSt != ST_IDLE) { pesan = String("ditolak: state ") + NAMA_ST[sSt] + " (uji hanya dari IDLE)"; return false; }
  pindah(ST_UJI);
  return true;
}

bool fsmUjiJalan(float v, float w, uint32_t durasiMs, String &pesan) {
  if (!fsmUjiMasuk(pesan)) return false;
  sMan.aktif = false;
  sUjiJalan = true;
  sUjiSampai = millis() + durasiMs;
  penggerakSetKecepatan(constrain(v, -V_MAKS, V_MAKS), constrain(w, -3.0f, 3.0f));
  pesan = "uji jalan";
  return true;
}

bool fsmUjiPutar(float derajat, String &pesan) {
  if (!fsmUjiMasuk(pesan)) return false;
  sUjiJalan = false;
  Langkah l[1] = {{L_PUTAR, derajat * DEG}};
  manuverMulai(l, 1, 0);
  pesan = "uji putar";
  return true;
}

bool fsmUjiLurus(float meter, String &pesan) {
  if (!fsmUjiMasuk(pesan)) return false;
  sUjiJalan = false;
  Langkah l[1] = {{meter >= 0 ? L_MAJU : L_MUNDUR, fabsf(meter)}};
  manuverMulai(l, 1, P.vBersih);
  pesan = "uji lurus";
  return true;
}

// =====================================================================
//  Logika CLEANING
// =====================================================================
static void mulaiBelokU(int8_t sisi) {
  sSisi = sisi;
  Langkah l[3] = {{L_PUTAR, sisi * 90.0f * DEG}, {L_MAJU, P.lebarLajur}, {L_PUTAR, sisi * 90.0f * DEG}};
  manuverMulai(l, 3, P.vDekat);
  sSub = SB_BELOK_U;
}

static void mulaiLajur() {
  Pose p = penggerakPose();
  sSLajurMulai = p.s;
  sSub = SB_LURUS;
}

static void selesaiBersih(const char *alasan) {
  Serial.printf("# SELESAI membersihkan: %s (%.1f menit)\n", alasan, (millis() - sTMulaiBersih) / 60000.0f);
  semuaAman(false);
  buzzerPola(2, 150, 150);
  pindah(ST_IDLE);
}

static void updateBersih(uint32_t now) {
  Pose p = penggerakPose();
  if (sMasukBaru) {
    sMasukBaru = false;
    if (sStPrev == ST_IDLE) {
      // START: {W} = pose saat ini setelah diam (bias gyro diperbarui), suction dinyalakan dulu
      sSub = SB_PERSIAPAN;
      sTMulaiBersih = now;
      penggerakBerhenti(false);
      if (P.suction >= 0.5f) suctionSet(true);
      imuMulaiKalibBias(75);   // ~1,5 s pada 50 Hz
      kameraInginDeteksi(P.modeKotor >= 0.5f);
      sGeserGagal = 0;
      sSisi = +1;
      sEpisodeKotor = false;
      sLambatSampaiS = -1e9f;
      sZonaUlangSampaiS = -1e9f;
      buzzerPola(1, 80, 0);
    }
  }
  ledSetPola(kameraKotor() ? LED_KOTOR : LED_BERSIH);

  if (now - sTMulaiBersih > (uint32_t)(P.durasiMaksMnt * 60000.0f)) { selesaiBersih("durasi maksimum"); return; }
  if (usRusak(US_DEPAN)) { fsmGalat("HC-SR04 depan rusak / tidak merespons saat membersihkan"); return; }

  switch (sSub) {
    case SB_PERSIAPAN:
      penggerakSetKecepatan(0, 0);
      if (!imuKalibBiasSibuk() && (P.suction < 0.5f || suctionLamaNyalaMs() >= SUCTION_SPINUP_MS) && penggerakDiam()) {
        penggerakResetOdo();      // origin {W} = pose START
        sLajurHeading = 0;
        mulaiLajur();
        Serial.println(F("# lajur pertama dimulai (heading 0)"));
      }
      break;

    case SB_LURUS: {
      if (depanDarurat()) { pindah(ST_HINDAR); return; }
      if (depanTerhalang() || (p.s - sSLajurMulai) > P.lajurMaks) {
        penggerakBerhenti(false);
        mulaiBelokU(sSisi);
        return;
      }
      // kotor: perlambat sampai nozzle melewati titik kotor terakhir (+ jarak kamera)
      if (P.modeKotor >= 0.5f && kameraKotor()) {
        if (!sEpisodeKotor) { sEpisodeKotor = true; sEpisodeDiulang = p.s < sZonaUlangSampaiS; sKotorMulaiS = p.s + P.jarakKamera; }
        sLambatSampaiS = p.s + P.jarakKamera;
      }
      bool lambatKotor = p.s < sLambatSampaiS;
      if (sEpisodeKotor && !lambatKotor) {
        // episode kotor selesai (nozzle sudah melewati kotor)
        sEpisodeKotor = false;
        if (P.modeKotor >= 1.5f && !sEpisodeDiulang) {
          float panjang = constrain(sLambatSampaiS - sKotorMulaiS + 0.10f, 0.10f, 0.50f);
          sEpisodeDiulang = true;
          Langkah l[1] = {{L_MUNDUR, panjang}};
          manuverMulai(l, 1, P.vKotor);
          sLambatSampaiS = p.s;            // akan diperbarui setelah mundur
          sKotorMulaiS = panjang;          // dipakai sebagai panjang lintasan ulang
          sSub = SB_ULANG_KOTOR;
          Serial.printf("# kotor: ulangi lintasan %.2f m\n", panjang);
          return;
        }
      }
      float v = P.vBersih;
      if (usCm(US_DEPAN) < P.dDepanLambat || tofMinMm() < 2.0f * P.dTofBelok) v = fminf(v, P.vDekat);
      if (lambatKotor) v = fminf(v, P.vKotor);
      float w = constrain(P.kHeading * bungkusPi(sLajurHeading - p.th), -0.8f, 0.8f);
      // menjauh dari dinding samping
      if (usCm(US_KIRI) < P.dSamping) w -= 0.3f;
      if (usCm(US_KANAN) < P.dSamping) w += 0.3f;
      penggerakSetKecepatan(v, w);
      break;
    }

    case SB_ULANG_KOTOR: {
      HasilManuver h = manuverUpdate(now);
      if (h == M_SELESAI) {
        Pose q = penggerakPose();
        sLambatSampaiS = q.s + sKotorMulaiS;   // jalan pelan lagi sepanjang lintasan ulang
        sZonaUlangSampaiS = sLambatSampaiS + P.jarakKamera;
        sEpisodeKotor = false;
        sSub = SB_LURUS;
      } else if (h != M_JALAN) {
        sSub = SB_LURUS;
      }
      break;
    }

    case SB_BELOK_U: {
      HasilManuver h = manuverUpdate(now);
      if (h == M_SELESAI) {
        sLajurHeading = bungkusPi(sLajurHeading + PI);
        sSisi = -sSisi;   // sisi relatif terhadap heading baru berbalik (arah geser dunia tetap)
        mulaiLajur();
      } else if (h == M_TERHALANG) {
        // geser ke samping terhalang: sisi ini sudah habis, coba arah sebaliknya
        if (++sGeserGagal >= 2) { selesaiBersih("area habis (geser lajur terhalang di kedua sisi)"); return; }
        Serial.println(F("# geser lajur terhalang, pindah ke sisi sebaliknya"));
        int8_t s = sSisi;
        Langkah l[3] = {{L_PUTAR, 180.0f * DEG}, {L_MAJU, P.lebarLajur}, {L_PUTAR, -s * 90.0f * DEG}};
        manuverMulai(l, 3, P.vDekat);
        sSisi = -s;
        // setelah selesai: lajur berlawanan dengan lajur sebelumnya (sama dengan belok-U biasa)
      } else if (h == M_TIMEOUT) {
        pindah(ST_HINDAR);
      }
      break;
    }
  }
}

static void updateHindar(uint32_t now) {
  if (sMasukBaru) {
    sMasukBaru = false;
    ledSetPola(LED_HINDAR);
    if (catatRiwayat(sRiwayatHindar, 4, 15000)) { fsmGalat("terjebak: 4 kali hindar dalam 15 s"); return; }
    Langkah l[1] = {{L_MUNDUR, MUNDUR_HINDAR}};
    manuverMulai(l, 1, 0);
  }
  HasilManuver h = manuverUpdate(now);
  if (h == M_JALAN) return;
  // pilih sisi yang lebih lega untuk belok-U
  float ki = usCm(US_KIRI), ka = usCm(US_KANAN);
  int tKi = tofJarakMm(TOF_KI), tKa = tofJarakMm(TOF_KA);
  if (tKi >= 0) ki = fminf(ki, tKi / 10.0f * 1.4f);   // serong 45 derajat: proyeksi kasar ke samping
  if (tKa >= 0) ka = fminf(ka, tKa / 10.0f * 1.4f);
  int8_t sisi = (ki >= ka) ? +1 : -1;
  Serial.printf("# hindar: kiri %.0f cm, kanan %.0f cm -> belok-U ke %s\n", ki, ka, sisi > 0 ? "kiri" : "kanan");
  sStPrev = ST_HINDAR;
  sSt = ST_BERSIH;          // kembali ke CLEANING tanpa inisialisasi ulang
  sMasukBaru = false;
  Serial.println(F("# STATE HINDAR -> BERSIH"));
  // belok-U pertama: putar 90 ke sisi lega, geser, putar 90 -> lajur baru berlawanan
  mulaiBelokU(sisi);
}

static void updateTebing(uint32_t now) {
  if (!sMasukBaru && sTebingFase == 2 && amanTebing()) { sMasukBaru = true; sTMasuk = now; }   // tebing baru saat berputar
  if (sMasukBaru) {
    sMasukBaru = false;
    ledSetPola(LED_HINDAR);
    buzzerPola(1, 60, 0);
    sTebingKiAwal = amanTebingKi();
    sTebingKaAwal = amanTebingKa();
    sTebingMundurTotal = 0;
    sTebingFase = 0;
    Serial.printf("# TEBING terdeteksi: kiri %d kanan %d\n", sTebingKiAwal, sTebingKaAwal);
    if (catatRiwayat(sRiwayatTebing, 5, 20000)) { fsmGalat("tebing berulang: 5 kali dalam 20 s"); return; }
    penggerakBerhenti(true);
  }
  switch (sTebingFase) {
    case 0:   // tunggu berhenti (rem darurat sudah berjalan lewat batas LARANG_MAJU)
      if (penggerakDiam() || now - sTMasuk > 800) {
        Langkah l[1] = {{L_MUNDUR, P.mundurTebing}};
        manuverMulai(l, 1, 0);
        sTebingMundurTotal = P.mundurTebing;
        sTebingFase = 1;
      }
      break;
    case 1: {
      HasilManuver h = manuverUpdate(now);
      if (h == M_JALAN) break;
      if (amanTebing()) {
        if (sTebingMundurTotal + 0.05f > MUNDUR_TEBING_MAKS) { fsmGalat("tebing tetap terbaca setelah mundur maksimum"); return; }
        Langkah l[1] = {{L_MUNDUR, 0.05f}};
        manuverMulai(l, 1, 0);
        sTebingMundurTotal += 0.05f;
        break;
      }
      // putar menjauhi tebing: kiri -> putar kanan, kanan -> putar kiri, keduanya -> balik 180
      float sudut = (sTebingKiAwal && sTebingKaAwal) ? 180.0f : (sTebingKiAwal ? -120.0f : 120.0f);
      Langkah l[1] = {{L_PUTAR, sudut * DEG}};
      manuverMulai(l, 1, 0);
      sTebingFase = 2;
      break;
    }
    case 2: {
      HasilManuver h = manuverUpdate(now);
      if (h == M_JALAN) break;
      if (h == M_TIMEOUT) { fsmGalat("putar setelah tebing tidak selesai (roda tertahan?)"); return; }
      // lajur baru searah heading sekarang
      Pose p = penggerakPose();
      sLajurHeading = p.th;
      sStPrev = ST_TEBING;
      sSt = ST_BERSIH;
      sMasukBaru = false;
      Serial.println(F("# STATE TEBING -> BERSIH (lajur baru)"));
      mulaiLajur();
      break;
    }
  }
}

static void updateUji(uint32_t now) {
  if (sMasukBaru) { sMasukBaru = false; ledSetPola(LED_UJI); sUjiJalan = false; }
  if (sUjiJalan && (int32_t)(now - sUjiSampai) >= 0) {
    sUjiJalan = false;
    penggerakBerhenti(false);
    Serial.println(F("# uji jalan selesai"));
  }
  if (sMan.aktif) {
    HasilManuver h = manuverUpdate(now);
    if (h != M_JALAN) {
      Pose p = penggerakPose();
      Serial.printf("# uji manuver selesai: hasil %d (1 selesai, 2 terhalang, 3 timeout), pose x %.3f y %.3f th %.1f deg\n",
                    h, p.x, p.y, p.th / DEG);
      penggerakBerhenti(false);
    }
  }
}

// =====================================================================
void fsmUpdate(uint32_t now) {
  uint16_t k = amanKondisi();

  // ---------- Prioritas keselamatan (menimpa state apa pun) ----------
  if (sSt != ST_GALAT && sSt != ST_BOOT) {
    if (penggerakFault()) { semuaAman(true); fsmGalat(penggerakAlasanFault()); }
    else if ((k & KA_MIRING) && stateBergerak()) { semuaAman(true); fsmGalat("robot miring / terangkat (akselerometer)"); }
  }
  if (sSt != ST_GALAT && sSt != ST_BATERAI && sSt != ST_BOOT && (k & KA_BAT_KRITIS)) {
    semuaAman(true);
    pindah(ST_BATERAI);
  }
  if ((sSt == ST_BERSIH || sSt == ST_HINDAR) && amanTebing()) {
    sMan.aktif = false;
    pindah(ST_TEBING);
  }

  switch (sSt) {
    case ST_BOOT:
      if (sMasukBaru) { sMasukBaru = false; ledSetPola(LED_BOOT); imuMulaiKalibBias(75); }
      if (now - sTMasuk > 2000 && !imuKalibBiasSibuk()) {
        Serial.printf("# kamera: %s\n", kameraAda() ? "ADA" : "TIDAK ADA (robot tetap bisa jalan tanpa deteksi kotor)");
        buzzerPola(1, 100, 0);
        pindah(ST_IDLE);
      }
      break;

    case ST_IDLE:
      if (sMasukBaru) { sMasukBaru = false; semuaAman(false); }
      ledSetPola(dayaStatus() == BAT_PERINGATAN ? LED_PERINGATAN : LED_SIAGA);
      penggerakSetKecepatan(0, 0);
      break;

    case ST_BERSIH:
      updateBersih(now);
      if (sSt == ST_BERSIH && dayaStatus() == BAT_PERINGATAN && (now / 1000) % 30 == 0 && (now % 1000) < 100)
        buzzerPola(1, 40, 0);   // pengingat baterai menipis tiap 30 s
      break;

    case ST_HINDAR: updateHindar(now); break;
    case ST_TEBING: updateTebing(now); break;

    case ST_BATERAI:
      if (sMasukBaru) {
        sMasukBaru = false;
        semuaAman(true);
        ledSetPola(LED_BATERAI);
        buzzerPola(3, 300, 200, 10000);   // 3 bunyi tiap 10 s
        Serial.printf("# BATERAI HABIS: VBAT %.2f V < %.2f V. Semua aktuator mati. Isi baterai (lepas bila > %.2f V selama 10 s).\n",
                      dayaVbat(), P.vbatStop, P.vbatPulih);
      }
      penggerakSetKecepatan(0, 0);
      if (dayaStatus() != BAT_KRITIS) { buzzerDiam(); buzzerPola(2, 100, 100); pindah(ST_IDLE); }
      break;

    case ST_GALAT:
      if (sMasukBaru) {
        sMasukBaru = false;
        semuaAman(true);
        ledSetPola(LED_GALAT);
        buzzerPola(1, 800, 0, 5000);      // 1 bunyi panjang tiap 5 s
        Serial.println(F("# Ketik 'status' untuk detail, 'reset' setelah penyebab diperbaiki."));
      }
      penggerakSetKecepatan(0, 0);
      break;

    case ST_UJI: updateUji(now); break;
  }
}

void fsmTiapDetik(uint32_t now) {
  (void)now;
  if (sSt != ST_IDLE || P.jadwalAktif < 0.5f || !rtcAda()) return;
  WaktuRtc w;
  if (!rtcBaca(w) || !w.valid) return;
  uint16_t kodeTgl = (uint16_t)(w.bln * 32 + w.tgl);
  if (w.jam == (uint8_t)P.jadwalJam && w.mnt == (uint8_t)P.jadwalMenit && kodeTgl != sJadwalTglTerakhir) {
    sJadwalTglTerakhir = kodeTgl;
    String pesan;
    fsmPerintahMulai("jadwal RTC", pesan);
    Serial.printf("# jadwal %02u:%02u: %s\n", w.jam, w.mnt, pesan.c_str());
  }
}

void fsmCetak(Stream &s) {
  Pose p = penggerakPose();
  s.printf("# state %s / %s, lajur heading %.0f deg, sisi %+d, waktu bersih %.1f menit\n", NAMA_ST[sSt], fsmNamaSub(),
           sLajurHeading / DEG, sSisi, sSt == ST_BERSIH ? (millis() - sTMulaiBersih) / 60000.0f : 0.0f);
  if (sSt == ST_GALAT) s.printf("# alasan galat: %s\n", sAlasanGalat);
  (void)p;
}
