// kamera_link.cpp - pengurai bingkai + pemeliharaan sambungan kamera
#include "kamera_link.h"
#include "config.h"
#include "param.h"
#include "protokol_kamera.h"

static HardwareSerial &LINK = Serial1;
static PkPengurai sPengurai;
static InfoKamera sInfo;
static uint8_t sSeq = 0;
static bool sIngin = false;
static uint32_t sTPelihara = 0, sTPing = 0;
static bool sEcho = false;

static void kirim(uint8_t tipe, const uint8_t *payload, uint8_t len) {
  uint8_t buf[PK_MAKS_BINGKAI];
  size_t n = pkBangun(buf, tipe, sSeq++, payload, len);
  if (n) LINK.write(buf, n);
}

void kameraKirimPing() { kirim(PK_PING, nullptr, 0); }
void kameraKirimMulai(uint16_t periodeMs) { uint8_t p[2]; pkTulisU16(p, periodeMs); kirim(PK_MULAI, p, 2); }
void kameraKirimBerhenti() { kirim(PK_BERHENTI, nullptr, 0); }
void kameraKirimMintaStatus() { kirim(PK_MINTA_STATUS, nullptr, 0); }
void kameraKirimSetParam(uint8_t id, int16_t nilai) {
  uint8_t p[3] = {id, (uint8_t)(nilai & 0xFF), (uint8_t)((uint16_t)nilai >> 8)};
  kirim(PK_SET_PARAM, p, 3);
}
void kameraKirimKalibrasi() { kirim(PK_KALIBRASI, nullptr, 0); }
void kameraKirimHapusKalib() { kirim(PK_HAPUS_KALIB, nullptr, 0); }
void kameraSetEcho(bool aktif) { sEcho = aktif; }

void kameraMulai() {
  memset(&sInfo, 0, sizeof(sInfo));
  LINK.setRxBufferSize(1024);
  LINK.begin(PK_BAUD, SERIAL_8N1, PIN_CAM_RX, PIN_CAM_TX);
}

static void prosesBingkai(uint32_t now) {
  const uint8_t *p = sPengurai.payload;
  uint8_t n = sPengurai.len;
  sInfo.tRxMs = now;
  switch (sPengurai.tipe) {
    case PK_HEARTBEAT:
      if (n >= 8) {
        sInfo.flags = p[0];
        sInfo.fps = pkBacaU16(p + 1) / 10.0f;
        sInfo.sensorPid = pkBacaU16(p + 3);
        sInfo.errFrame = p[5];
        sInfo.uptimeS = pkBacaU16(p + 6);
      }
      break;
    case PK_STATUS:
      if (n >= 12) {
        sInfo.flags = p[0];
        sInfo.floorMean = p[1]; sInfo.floorStd = p[2]; sInfo.darkThr = p[3];
        sInfo.normalDark = pkBacaU16(p + 4); sInfo.dirtyLimit = pkBacaU16(p + 6);
        sInfo.periodeMs = pkBacaU16(p + 8); sInfo.progresKalib = p[10]; sInfo.papan = p[11];
        Serial.printf("# KAMERA STATUS: flags 0x%02X lantai %u (sd %u) ambang gelap %u normal %u px batas %u px periode %u ms kalib %u%%\n",
                      sInfo.flags, sInfo.floorMean, sInfo.floorStd, sInfo.darkThr, sInfo.normalDark,
                      sInfo.dirtyLimit, sInfo.periodeMs, sInfo.progresKalib);
      }
      break;
    case PK_KOTOR:
      if (n >= 12) {
        sInfo.skor = p[0]; sInfo.kflags = p[1]; sInfo.region = p[2];
        sInfo.skorKol[0] = p[3]; sInfo.skorKol[1] = p[4]; sInfo.skorKol[2] = p[5];
        sInfo.dark = pkBacaU16(p + 6); sInfo.limit = pkBacaU16(p + 8);
        sInfo.mean = p[10]; sInfo.nomorFrame = p[11];
        sInfo.tKotorMs = now;
        if (sEcho)
          Serial.printf("# KOTOR:%d skor %u region %c%c%c dark %u batas %u mean %u%s\n",
                        (sInfo.kflags & PKK_KOTOR) ? 1 : 0, sInfo.skor,
                        (sInfo.region & PKR_KIRI) ? 'L' : '-', (sInfo.region & PKR_TENGAH) ? 'C' : '-',
                        (sInfo.region & PKR_KANAN) ? 'R' : '-', sInfo.dark, sInfo.limit, sInfo.mean,
                        (sInfo.kflags & PKK_TERKALIBRASI) ? "" : " (belum kalibrasi)");
      }
      break;
    case PK_ACK:
      if (n >= 3) {
        sInfo.ackTipe = p[0]; sInfo.ackParam = p[1]; sInfo.ackStatus = p[2];
        if (p[2] != PKA_OK || p[0] == PK_SET_PARAM || p[0] == PK_KALIBRASI || p[0] == PK_HAPUS_KALIB)
          Serial.printf("# KAMERA ACK perintah 0x%02X param 0x%02X status %u%s\n", p[0], p[1], p[2], p[2] ? " (GAGAL)" : " (OK)");
      }
      break;
    case PK_GALAT:
      if (n >= 2) {
        sInfo.galatKode = p[0]; sInfo.galatDetail = p[1];
        Serial.printf("# KAMERA GALAT kode %u detail %u\n", p[0], p[1]);
      }
      break;
    default:
      break;
  }
}

void kameraPoll(uint32_t now) {
  int budget = 256;   // batasi kerja per panggilan
  while (LINK.available() && budget--) {
    int r = sPengurai.masukkan((uint8_t)LINK.read());
    if (r == 1) prosesBingkai(now);
    // r == -1: teks/log dari kamera (boot ROM, debug) -> diabaikan
  }
  sInfo.nBingkai = sPengurai.nOk;
  sInfo.nCrcSalah = sPengurai.nCrcSalah;

  // PING bila tidak ada bingkai > 1,5 s (heartbeat normalnya tiap 1 s)
  if (now - sInfo.tRxMs > 1500 && now - sTPing > 1000) { sTPing = now; kameraKirimPing(); }

  // Pemeliharaan tiap 1 s: samakan status deteksi kamera dengan keinginan FSM (kamera bisa reboot)
  if (now - sTPelihara >= 1000) {
    sTPelihara = now;
    if (kameraAda()) {
      bool aktif = sInfo.flags & PKF_DETEKSI_AKTIF;
      if (sIngin && !aktif) kameraKirimMulai(KAMERA_PERIODE_MS);
      else if (!sIngin && aktif) kameraKirimBerhenti();
    }
  }
}

bool kameraAda() { return sInfo.tRxMs != 0 && millis() - sInfo.tRxMs < KAMERA_ABSEN_MS; }

void kameraInginDeteksi(bool aktif) {
  if (aktif != sIngin) {
    sIngin = aktif;
    if (kameraAda()) { if (aktif) kameraKirimMulai(KAMERA_PERIODE_MS); else kameraKirimBerhenti(); }
  }
}

bool kameraKotor() {
  if (!kameraAda()) return false;
  if (millis() - sInfo.tKotorMs > 1000) return false;            // hasil basi
  if (P.kotorPerluKalib >= 0.5f && !(sInfo.kflags & PKK_TERKALIBRASI)) return false;
  return sInfo.kflags & PKK_KOTOR;
}

const InfoKamera &kameraInfo() { return sInfo; }

void kameraCetak(Stream &s) {
  if (!kameraAda()) {
    s.printf("# KAMERA: TIDAK ADA (bingkai terakhir %s), bingkai OK %lu, CRC salah %lu. Robot tetap jalan tanpa deteksi kotor.\n",
             sInfo.tRxMs ? "lebih dari 3 s lalu" : "belum pernah", (unsigned long)sInfo.nBingkai, (unsigned long)sInfo.nCrcSalah);
    return;
  }
  const char *sensor = sInfo.sensorPid == 0x26 ? "OV2640" : sInfo.sensorPid == 0x3660 ? "OV3660" : "lain";
  s.printf("# KAMERA: ada, sensor %s (0x%04X), %.1f fps, flags 0x%02X [%s%s%s%s], uptime %u s, err frame %u, bingkai %lu, CRC salah %lu\n",
           sensor, sInfo.sensorPid, sInfo.fps, sInfo.flags,
           (sInfo.flags & PKF_KAMERA_OK) ? "kamera-ok " : "KAMERA-GAGAL ",
           (sInfo.flags & PKF_DETEKSI_AKTIF) ? "deteksi " : "",
           (sInfo.flags & PKF_TERKALIBRASI) ? "terkalibrasi " : "BELUM-KALIBRASI ",
           (sInfo.flags & PKF_KALIBRASI_JALAN) ? "kalibrasi-jalan" : "",
           sInfo.uptimeS, sInfo.errFrame, (unsigned long)sInfo.nBingkai, (unsigned long)sInfo.nCrcSalah);
  s.printf("# KOTOR terakhir: %d skor %u region 0x%02X (L%u C%u R%u) dark %u batas %u mean %u, %lu ms lalu\n",
           (sInfo.kflags & PKK_KOTOR) ? 1 : 0, sInfo.skor, sInfo.region, sInfo.skorKol[0], sInfo.skorKol[1],
           sInfo.skorKol[2], sInfo.dark, sInfo.limit, sInfo.mean,
           sInfo.tKotorMs ? (unsigned long)(millis() - sInfo.tKotorMs) : 0UL);
}
