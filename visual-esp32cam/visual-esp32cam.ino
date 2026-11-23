// =====================================================================
//  FIRMWARE MODUL VISUAL - deteksi area kotor (Robot Pembersih Lantai Otonom)
//  Papan : AI-Thinker ESP32-CAM ATAU klon ESP32-S3-CAM (pilih di papan_kamera.h), sensor OV3660/OV2640
//  Core  : arduino-esp32 3.x (diuji kompilasi 3.3.12), pustaka esp32-camera bawaan core
//  Link  : UART0 (U0R/U0T) 115200 8N1 ke ESP32-S3 utama, protokol biner protokol_kamera.h
//          (spesifikasi: 06 Implementasi\07 Integrasi\Protokol_UART_Kamera.md)
//
//  Metode (sama dengan uji dirt detection tim, lihat 07 Dokumentasi\Software\Arsitektur_Dirt_Detection.png):
//   KALIBRASI (sekali, di lantai bersih, robot diam):
//     Fase 1: 20 frame -> floorMean dan simpangan piksel ROI -> darkThr (batas gelap)
//     Fase 2: 20 frame -> jumlah dark pixel normal (normalDark, sd) -> dirtyLimit (batas kotor)
//     Hasil disimpan di NVS (namespace "dirt"), tetap setelah reboot.
//   RUNTIME (tiap 150 ms):
//     Frame 160x120 grayscale -> ROI lantai 120x65 -> hitung dark pixel (piksel < darkThr,
//     ambang digeser mengikuti rata-rata kecerahan frame bila kompensasi aktif) -> D > dirtyLimit = kandidat
//     -> histeresis 3 nyala / 3 mati -> kirim ke ESP32-S3 (pesan KOTOR: skor 0-100, flag, region kiri/tengah/kanan).
//
//  LED flash MATI secara bawaan (silau pada jendela kamera). Konsol teks (115200) menerima perintah
//  "bantuan" untuk uji di meja lewat USB-TTL / ESP32-CAM-MB.
// =====================================================================
#include <Arduino.h>
#include "esp_camera.h"
#include <Preferences.h>
#include "protokol_kamera.h"
#include "papan_kamera.h"

#define FW_VERSI "0.1.0"

// =====================================================================
//  KONFIGURASI (cocok dengan uji tim: frame 160x120, ROI 120x65 di bagian bawah)
// =====================================================================
static const framesize_t UKURAN_FRAME = FRAMESIZE_QQVGA;   // 160 x 120
static const int REF_W = 160, REF_H = 120;                 // ROI didefinisikan pada ukuran ini
static const int ROI_X0 = 20, ROI_Y0 = 50, ROI_W = 120, ROI_H = 65;   // [UKUR] sesuaikan setelah kamera terpasang
// Orientasi: -1 = otomatis (OV3660 vflip 1 seperti contoh Espressif, lainnya 0), 0/1 = paksa.
// [UKUR] cek dengan perintah "ascii": bagian ATAS gambar = lantai JAUH, kiri gambar = kiri robot.
static const int CAM_VFLIP = -1, CAM_HMIRROR = -1;
static const uint8_t KALIB_N1 = 20, KALIB_N2 = 20;         // frame per fase kalibrasi
static const int MARGIN_MIN_PX = 40;                       // batas kotor minimal normal + 40 px
static const int K_SD_BATAS = 4;                           // batas kotor minimal normal + 4 sd
static const int CAHAYA_ANEH = 40;                         // |mean - floorMean| > ini -> flag cahaya aneh
static const int KOMPENSASI_MAKS = 40;                     // geser ambang maksimum (level abu)
static const uint32_t HEARTBEAT_MS = 1000;

// =====================================================================
//  Konsol dan jalur link
// =====================================================================
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  // ESP32-S3 dengan USB CDC aktif: konsol di USB native, link di UART0 (terpisah)
  #define LINK Serial0
  #define KONSOL_TERPISAH 1
#else
  // Konsol dan link berbagi UART0. Teks diabaikan oleh ESP32-S3 (SOF 0xA5 tidak pernah muncul di ASCII).
  #define LINK Serial
  #define KONSOL_TERPISAH 0
#endif
#define KONSOL Serial

// =====================================================================
//  State
// =====================================================================
struct ParamDeteksi {
  uint16_t periodeMs = 150;
  int16_t kGelapX10 = 30;
  int16_t deltaGelap = 20;
  int16_t rasioBatasX100 = 145;
  int16_t histNyala = 3;
  int16_t histMati = 3;
  int16_t batasManual = 0;
  int16_t gelapManual = 0;
  int16_t kompensasi = 1;
  int16_t ledFlash = 0;
};

struct Kalibrasi {
  uint8_t ok = 0;
  float floorMean = 0, floorStd = 0;
  float normalDark = 0, sdDark = 0;
  float normalKol[3] = {0, 0, 0}, sdKol[3] = {0, 0, 0};
};

struct HasilFrame {   // hasil olah satu frame (ROI)
  float mean;
  double sum, sum2;
  uint32_t n;
  uint16_t dark;
  uint16_t darkKol[3];
};

static ParamDeteksi gP;
static Kalibrasi gK;
static bool gKameraOk = false;
static uint16_t gSensorPid = 0;
static bool gDeteksi = false;
static bool gDebug = false;
static uint8_t gErrFrame = 0;
static uint8_t gGagalBeruntun = 0;
static uint8_t gSeq = 0;
static uint8_t gNomorFrame = 0;
// histeresis
static uint8_t gHitNyala = 0, gHitMati = 0;
static bool gKotor = false;
// fps
static uint16_t gFpsX10 = 0;
static uint16_t gFrameDetik = 0;
static uint32_t gTFps = 0;
// kalibrasi berjalan
static uint8_t gKalibFase = 0;   // 0 = tidak, 1 = fase 1, 2 = fase 2
static uint8_t gKalibN = 0;
static double gKS1 = 0, gKS2 = 0, gKCnt = 0;           // fase 1: jumlah, jumlah kuadrat, cacah piksel
static double gKD1 = 0, gKD2 = 0;                       // fase 2: jumlah D, D^2
static double gKDk1[3] = {0, 0, 0}, gKDk2[3] = {0, 0, 0};
static uint8_t gKalibThr = 0;
// ROI terskala
static int rX0, rY0, rW, rH;
// pengurai
static PkPengurai gPgLink;
#if KONSOL_TERPISAH
static PkPengurai gPgKonsol;
#endif
static char gTeks[64];
static uint8_t gTeksLen = 0;

// =====================================================================
//  Kirim bingkai
// =====================================================================
static void kirim(uint8_t tipe, const uint8_t *p, uint8_t len) {
  uint8_t buf[PK_MAKS_BINGKAI];
  size_t n = pkBangun(buf, tipe, gSeq++, p, len);
  if (n) LINK.write(buf, n);
}

static uint8_t flagStatus() {
  uint8_t f = 0;
  if (gDeteksi) f |= PKF_DETEKSI_AKTIF;
  if (gK.ok) f |= PKF_TERKALIBRASI;
  if (gKalibFase) f |= PKF_KALIBRASI_JALAN;
  if (gKameraOk) f |= PKF_KAMERA_OK;
  if (gP.ledFlash) f |= PKF_LED_NYALA;
  return f;
}

static void kirimHeartbeat() {
  uint8_t p[8];
  p[0] = flagStatus();
  pkTulisU16(p + 1, gFpsX10);
  pkTulisU16(p + 3, gSensorPid);
  p[5] = gErrFrame;
  pkTulisU16(p + 6, (uint16_t)(millis() / 1000));
  kirim(PK_HEARTBEAT, p, 8);
}

static uint8_t ambangGelapKalib() {
  float d = fmaxf(gP.kGelapX10 / 10.0f * gK.floorStd, (float)gP.deltaGelap);
  return (uint8_t)constrain(gK.floorMean - d, 1.0f, 254.0f);
}

static uint16_t batasKotor(float normal, float sd, int marginMin) {
  float a = normal * gP.rasioBatasX100 / 100.0f;
  float b = normal + K_SD_BATAS * sd;
  float c = normal + marginMin;
  return (uint16_t)fminf(65535.0f, fmaxf(a, fmaxf(b, c)));
}

static uint16_t batasTotal() {
  if (gP.batasManual > 0) return gP.batasManual;
  if (gK.ok) return batasKotor(gK.normalDark, gK.sdDark, MARGIN_MIN_PX);
  return (uint16_t)(rW * rH * 5 / 100);   // tanpa kalibrasi: 5% ROI
}
static float normalTotal() { return gK.ok ? gK.normalDark : rW * rH * 2 / 100.0f; }   // tanpa kalibrasi: 2% ROI
static uint16_t batasKolom(uint8_t k) {
  if (gP.batasManual > 0) return gP.batasManual / 3;
  if (gK.ok) return batasKotor(gK.normalKol[k], gK.sdKol[k], MARGIN_MIN_PX / 3);
  return batasTotal() / 3;
}
static float normalKolom(uint8_t k) { return gK.ok ? gK.normalKol[k] : normalTotal() / 3.0f; }

static void kirimStatus() {
  uint8_t p[12];
  p[0] = flagStatus();
  p[1] = (uint8_t)constrain(gK.floorMean, 0.0f, 255.0f);
  p[2] = (uint8_t)constrain(gK.floorStd, 0.0f, 255.0f);
  p[3] = gK.ok ? ambangGelapKalib() : 0;
  pkTulisU16(p + 4, (uint16_t)normalTotal());
  pkTulisU16(p + 6, batasTotal());
  pkTulisU16(p + 8, gP.periodeMs);
  p[10] = gKalibFase == 0 ? (gK.ok ? 100 : 0) : (uint8_t)((gKalibFase - 1) * 50 + gKalibN * 50 / KALIB_N1);
  p[11] = PAPAN_ID;
  kirim(PK_STATUS, p, 12);
}

static void kirimAck(uint8_t tipe, uint8_t param, uint8_t status) {
  uint8_t p[3] = {tipe, param, status};
  kirim(PK_ACK, p, 3);
}

static void kirimGalat(uint8_t kode, uint8_t detail) {
  uint8_t p[2] = {kode, detail};
  kirim(PK_GALAT, p, 2);
}

// =====================================================================
//  NVS
// =====================================================================
static void simpanKalib() {
  Preferences pr;
  if (!pr.begin("dirt", false)) return;
  pr.putBytes("k", &gK, sizeof(gK));
  pr.end();
}
static void simpanParam() {
  Preferences pr;
  if (!pr.begin("dirt", false)) return;
  pr.putBytes("p", &gP, sizeof(gP));
  pr.end();
}
static void muatNvs() {
  Preferences pr;
  if (!pr.begin("dirt", true)) return;
  if (pr.getBytesLength("k") == sizeof(gK)) pr.getBytes("k", &gK, sizeof(gK));
  if (pr.getBytesLength("p") == sizeof(gP)) pr.getBytes("p", &gP, sizeof(gP));
  pr.end();
}

// =====================================================================
//  Kamera
// =====================================================================
static void setLed(bool nyala) {
#if PIN_LED_FLASH >= 0
  digitalWrite(PIN_LED_FLASH, nyala ? HIGH : LOW);
#else
  (void)nyala;
#endif
}

static bool mulaiKamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_D0; c.pin_d1 = CAM_PIN_D1; c.pin_d2 = CAM_PIN_D2; c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4; c.pin_d5 = CAM_PIN_D5; c.pin_d6 = CAM_PIN_D6; c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK;
  c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn = CAM_PIN_PWDN;
  c.pin_reset = CAM_PIN_RESET;
  c.xclk_freq_hz = 20000000;
  c.frame_size = UKURAN_FRAME;
  c.pixel_format = PIXFORMAT_GRAYSCALE;   // OV2640 dan OV3660 mendukung grayscale
  c.jpeg_quality = 12;
  if (psramFound()) {
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;     // selalu frame terbaru
  } else {
    c.fb_count = 1;                       // QQVGA grayscale 19 KB muat di DRAM
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }
  esp_err_t e = esp_camera_init(&c);
  if (e != ESP_OK) {
    KONSOL.printf("# init kamera grayscale GAGAL 0x%x, coba RGB565...\n", (unsigned)e);
    esp_camera_deinit();
    c.pixel_format = PIXFORMAT_RGB565;
    e = esp_camera_init(&c);
  }
  if (e != ESP_OK) {
    KONSOL.printf("# init kamera GAGAL 0x%x (cek peta pin papan_kamera.h, kabel pita kamera, daya 5V)\n", (unsigned)e);
    return false;
  }
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    gSensorPid = s->id.PID;
    int vf = CAM_VFLIP >= 0 ? CAM_VFLIP : (gSensorPid == OV3660_PID ? 1 : 0);
    int hm = CAM_HMIRROR >= 0 ? CAM_HMIRROR : 0;
    s->set_vflip(s, vf);
    s->set_hmirror(s, hm);
    s->set_exposure_ctrl(s, 1);   // eksposur otomatis; perubahan kecerahan dikompensasi di ambang
    s->set_gain_ctrl(s, 1);
    s->set_whitebal(s, 1);
    KONSOL.printf("# kamera OK: sensor PID 0x%04X (%s), vflip %d hmirror %d, PSRAM %s\n", gSensorPid,
                  gSensorPid == OV3660_PID ? "OV3660" : gSensorPid == OV2640_PID ? "OV2640" : "lain",
                  vf, hm, psramFound() ? "ada" : "tidak ada");
  }
  // buang beberapa frame awal (eksposur otomatis menyesuaikan)
  for (int i = 0; i < 8; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
  return true;
}

static void hitungRoi(int w, int h) {
  rX0 = ROI_X0 * w / REF_W; rY0 = ROI_Y0 * h / REF_H;
  rW = ROI_W * w / REF_W;   rH = ROI_H * h / REF_H;
  if (rX0 + rW > w) rW = w - rX0;
  if (rY0 + rH > h) rH = h - rY0;
}

// ambil nilai abu piksel (grayscale langsung; RGB565 dikonversi ke luma)
static inline uint8_t piksel(const camera_fb_t *fb, int x, int y) {
  if (fb->format == PIXFORMAT_GRAYSCALE) return fb->buf[y * fb->width + x];
  const uint8_t *p = fb->buf + 2 * (y * fb->width + x);
  uint16_t v = (p[0] << 8) | p[1];   // RGB565 big-endian dari esp32-camera
  uint8_t r = (v >> 11) << 3, g = ((v >> 5) & 0x3F) << 2, b = (v & 0x1F) << 3;
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

// rata-rata ROI (dan jumlah kuadrat untuk kalibrasi fase 1)
static void statistikRoi(const camera_fb_t *fb, HasilFrame &h) {
  h.sum = 0; h.sum2 = 0; h.n = 0;
  for (int y = rY0; y < rY0 + rH; y++)
    for (int x = rX0; x < rX0 + rW; x++) { uint8_t v = piksel(fb, x, y); h.sum += v; h.sum2 += (double)v * v; h.n++; }
  h.mean = h.n ? h.sum / h.n : 0;
}

static void hitungGelap(const camera_fb_t *fb, uint8_t thr, HasilFrame &h) {
  h.dark = 0;
  h.darkKol[0] = h.darkKol[1] = h.darkKol[2] = 0;
  int lebarKol = rW / 3;
  for (int y = rY0; y < rY0 + rH; y++) {
    for (int x = rX0; x < rX0 + rW; x++) {
      if (piksel(fb, x, y) < thr) {
        h.dark++;
        int k = (x - rX0) / (lebarKol > 0 ? lebarKol : 1);
        h.darkKol[k > 2 ? 2 : k]++;
      }
    }
  }
}

// ambang gelap efektif untuk frame ini
static uint8_t ambangEfektif(float meanFrame) {
  if (gP.gelapManual > 0) return (uint8_t)gP.gelapManual;
  if (!gK.ok) {   // tanpa kalibrasi: relatif terhadap rata-rata frame, sebaran diasumsikan 8
    float d = fmaxf(gP.kGelapX10 / 10.0f * 8.0f, (float)gP.deltaGelap);
    return (uint8_t)constrain(meanFrame - d, 1.0f, 254.0f);
  }
  float thr = ambangGelapKalib();
  if (gP.kompensasi) thr += constrain(meanFrame - gK.floorMean, (float)-KOMPENSASI_MAKS, (float)KOMPENSASI_MAKS);
  return (uint8_t)constrain(thr, 1.0f, 254.0f);
}

static uint8_t skor(float d, float normal, float batas) {
  float rentang = batas - normal;
  if (rentang < 1) rentang = 1;
  return (uint8_t)constrain(50.0f * (d - normal) / rentang, 0.0f, 100.0f);   // 50 = tepat di batas
}

// =====================================================================
//  Kalibrasi (non-blocking, satu frame per panggilan)
// =====================================================================
static void mulaiKalibrasi() {
  gKalibFase = 1; gKalibN = 0;
  gKS1 = gKS2 = gKCnt = 0;
  gKD1 = gKD2 = 0;
  for (int k = 0; k < 3; k++) gKDk1[k] = gKDk2[k] = 0;
  KONSOL.println(F("# KALIBRASI mulai: robot DIAM di lantai BERSIH (fase 1: warna lantai, fase 2: dark pixel normal)"));
}

static void langkahKalibrasi(const camera_fb_t *fb) {
  HasilFrame h;
  statistikRoi(fb, h);
  if (gKalibFase == 1) {
    gKS1 += h.sum; gKS2 += h.sum2; gKCnt += h.n;
    if (++gKalibN >= KALIB_N1) {
      gK.floorMean = gKS1 / gKCnt;
      gK.floorStd = sqrt(fmax(0.0, gKS2 / gKCnt - (double)gK.floorMean * gK.floorMean));
      if (gK.floorMean < 20 || gK.floorMean > 235 || gK.floorStd > 40) {
        KONSOL.printf("# KALIBRASI GAGAL: lantai mean %.1f sd %.1f (terlalu gelap/terang/tidak seragam)\n", gK.floorMean, gK.floorStd);
        gKalibFase = 0;
        muatNvs();   // kembalikan kalibrasi lama (bila ada)
        kirimGalat(PKG_KALIBRASI, 1);
        kirimStatus();
        return;
      }
      gKalibThr = ambangGelapKalib();
      KONSOL.printf("# fase 1 selesai: floorMean %.1f sd %.1f -> darkThr %u\n", gK.floorMean, gK.floorStd, gKalibThr);
      gKalibFase = 2; gKalibN = 0;
    }
  } else if (gKalibFase == 2) {
    float thr = gKalibThr;
    if (gP.kompensasi) thr += constrain(h.mean - gK.floorMean, (float)-KOMPENSASI_MAKS, (float)KOMPENSASI_MAKS);
    hitungGelap(fb, (uint8_t)constrain(thr, 1.0f, 254.0f), h);
    gKD1 += h.dark; gKD2 += (double)h.dark * h.dark;
    for (int k = 0; k < 3; k++) { gKDk1[k] += h.darkKol[k]; gKDk2[k] += (double)h.darkKol[k] * h.darkKol[k]; }
    if (++gKalibN >= KALIB_N2) {
      gK.normalDark = gKD1 / KALIB_N2;
      gK.sdDark = sqrt(fmax(0.0, gKD2 / KALIB_N2 - (double)gK.normalDark * gK.normalDark));
      for (int k = 0; k < 3; k++) {
        gK.normalKol[k] = gKDk1[k] / KALIB_N2;
        gK.sdKol[k] = sqrt(fmax(0.0, gKDk2[k] / KALIB_N2 - (double)gK.normalKol[k] * gK.normalKol[k]));
      }
      gK.ok = 1;
      gKalibFase = 0;
      gHitNyala = gHitMati = 0; gKotor = false;
      simpanKalib();
      KONSOL.printf("# KALIBRASI SELESAI: normalDark %.0f px (sd %.1f), dirtyLimit %u px, kolom normal %.0f/%.0f/%.0f. Tersimpan.\n",
                    gK.normalDark, gK.sdDark, batasTotal(), gK.normalKol[0], gK.normalKol[1], gK.normalKol[2]);
      kirimStatus();
    }
  }
}

// =====================================================================
//  Deteksi satu frame
// =====================================================================
static void deteksi(const camera_fb_t *fb) {
  HasilFrame h;
  statistikRoi(fb, h);
  uint8_t thr = ambangEfektif(h.mean);
  hitungGelap(fb, thr, h);
  uint16_t batas = batasTotal();
  float normal = normalTotal();
  bool kandidat = h.dark > batas;

  // histeresis (3 nyala / 3 mati)
  if (kandidat) { gHitMati = 0; if (gHitNyala < 255) gHitNyala++; if (gHitNyala >= gP.histNyala) gKotor = true; }
  else { gHitNyala = 0; if (gHitMati < 255) gHitMati++; if (gHitMati >= gP.histMati) gKotor = false; }

  uint8_t region = 0, sk[3];
  for (uint8_t k = 0; k < 3; k++) {
    uint16_t bk = batasKolom(k);
    if (h.darkKol[k] > bk) region |= (1 << k);   // bit0 kiri, bit1 tengah, bit2 kanan (kiri gambar = kiri robot)
    sk[k] = skor(h.darkKol[k], normalKolom(k), bk);
  }
  uint8_t fl = 0;
  if (gKotor) fl |= PKK_KOTOR;
  if (kandidat) fl |= PKK_KANDIDAT;
  if (gK.ok) fl |= PKK_TERKALIBRASI;
  if (gK.ok && fabsf(h.mean - gK.floorMean) > CAHAYA_ANEH) fl |= PKK_CAHAYA_ANEH;

  uint8_t p[12];
  p[0] = skor(h.dark, normal, batas);
  p[1] = fl;
  p[2] = region;
  p[3] = sk[0]; p[4] = sk[1]; p[5] = sk[2];
  pkTulisU16(p + 6, h.dark);
  pkTulisU16(p + 8, batas);
  p[10] = (uint8_t)constrain(h.mean, 0.0f, 255.0f);
  p[11] = gNomorFrame++;
  kirim(PK_KOTOR, p, 12);

  if (gDebug)   // format sama dengan log uji tim
    KONSOL.printf("DIRT:%d dark:%u px (normal %.0f, batas %u) mean:%.1f (lantai %.1f) thr:%u skor:%u region:%c%c%c\n",
                  gKotor ? 1 : 0, h.dark, normal, batas, h.mean, gK.floorMean, thr, p[0],
                  (region & 1) ? 'L' : '-', (region & 2) ? 'C' : '-', (region & 4) ? 'R' : '-');
}

// Gambar ASCII untuk mengarahkan kamera (tanpa Wi-Fi). ROI ditandai '|' dan '-'.
static void cetakAscii() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) { KONSOL.println(F("# gagal ambil frame")); return; }
  hitungRoi(fb->width, fb->height);
  static const char *G = " .:-=+*#%@";
  int sx = fb->width / 40, sy = fb->height / 24;
  if (sx < 1) sx = 1;
  if (sy < 1) sy = 1;
  KONSOL.printf("# frame %dx%d, ROI x%d y%d %dx%d (atas gambar = jauh)\n", fb->width, fb->height, rX0, rY0, rW, rH);
  for (int y = 0; y + sy <= (int)fb->height; y += sy) {
    char baris[48];
    int i = 0;
    for (int x = 0; x + sx <= (int)fb->width && i < 46; x += sx) {
      bool tepiX = (x <= rX0 && rX0 < x + sx) || (x <= rX0 + rW && rX0 + rW < x + sx);
      bool tepiY = (y <= rY0 && rY0 < y + sy) || (y <= rY0 + rH && rY0 + rH < y + sy);
      bool dalamX = x >= rX0 && x < rX0 + rW, dalamY = y >= rY0 && y < rY0 + rH;
      if (tepiY && dalamX) baris[i++] = '-';
      else if (tepiX && dalamY) baris[i++] = '|';
      else baris[i++] = G[piksel(fb, x, y) * 10 / 256];
    }
    baris[i] = 0;
    KONSOL.println(baris);
  }
  esp_camera_fb_return(fb);
}

// =====================================================================
//  Perintah dari ESP32-S3 (biner) dan konsol (teks)
// =====================================================================
static bool setParam(uint8_t id, int16_t v) {
  switch (id) {
    case PKP_PERIODE_MS:      if (v < 50 || v > 1000) return false; gP.periodeMs = v; break;
    case PKP_K_GELAP_X10:     if (v < 5 || v > 100) return false; gP.kGelapX10 = v; break;
    case PKP_DELTA_GELAP:     if (v < 5 || v > 120) return false; gP.deltaGelap = v; break;
    case PKP_RASIO_BATAS_X100:if (v < 105 || v > 400) return false; gP.rasioBatasX100 = v; break;
    case PKP_HIST_NYALA:      if (v < 1 || v > 20) return false; gP.histNyala = v; break;
    case PKP_HIST_MATI:       if (v < 1 || v > 20) return false; gP.histMati = v; break;
    case PKP_BATAS_MANUAL:    if (v < 0) return false; gP.batasManual = v; break;
    case PKP_GELAP_MANUAL:    if (v < 0 || v > 255) return false; gP.gelapManual = v; break;
    case PKP_KOMPENSASI:      if (v < 0 || v > 1) return false; gP.kompensasi = v; break;
    case PKP_LED_FLASH:       if (v < 0 || v > 1) return false; gP.ledFlash = v; setLed(v); break;
    default: return false;
  }
  simpanParam();
  return true;
}

static void prosesBingkai(const PkPengurai &pg) {
  const uint8_t *p = pg.payload;
  switch (pg.tipe) {
    case PK_PING: kirimHeartbeat(); break;
    case PK_MULAI:
      if (pg.len >= 2) { uint16_t per = pkBacaU16(p); if (per >= 50 && per <= 1000) gP.periodeMs = per; }
      gDeteksi = gKameraOk;
      kirimAck(PK_MULAI, 0, gKameraOk ? PKA_OK : PKA_KAMERA_GAGAL);
      break;
    case PK_BERHENTI: gDeteksi = false; kirimAck(PK_BERHENTI, 0, PKA_OK); break;
    case PK_MINTA_STATUS: kirimStatus(); break;
    case PK_SET_PARAM:
      if (pg.len < 3) { kirimAck(PK_SET_PARAM, 0, PKA_NILAI_SALAH); break; }
      kirimAck(PK_SET_PARAM, p[0], setParam(p[0], (int16_t)pkBacaU16(p + 1)) ? PKA_OK : PKA_NILAI_SALAH);
      break;
    case PK_KALIBRASI:
      if (!gKameraOk) { kirimAck(PK_KALIBRASI, 0, PKA_KAMERA_GAGAL); break; }
      if (gKalibFase) { kirimAck(PK_KALIBRASI, 0, PKA_SIBUK); break; }
      mulaiKalibrasi();
      kirimAck(PK_KALIBRASI, 0, PKA_OK);
      break;
    case PK_HAPUS_KALIB:
      gK = Kalibrasi();
      simpanKalib();
      kirimAck(PK_HAPUS_KALIB, 0, PKA_OK);
      break;
    default: kirimAck(pg.tipe, 0, PKA_TIDAK_DIKENAL); break;
  }
}

static void bantuan() {
  KONSOL.println(F("# Perintah konsol: bantuan | status | mulai | berhenti | kalib | hapus | debug 0|1 | led 0|1 |"));
  KONSOL.println(F("#   set <id> <nilai> (id lihat Protokol_UART_Kamera.md, mis. set 4 160) | ascii (gambar untuk mengarahkan kamera)"));
}

static void cetakStatus() {
  KONSOL.printf("# %s v%s, papan %s, kamera %s PID 0x%04X, %.1f fps, deteksi %s, err frame %u\n", "visual-esp32cam", FW_VERSI,
                PAPAN_NAMA, gKameraOk ? "OK" : "GAGAL", gSensorPid, gFpsX10 / 10.0f, gDeteksi ? "AKTIF" : "berhenti", gErrFrame);
  KONSOL.printf("# kalibrasi %s: floorMean %.1f sd %.1f darkThr %u normalDark %.0f (sd %.1f) dirtyLimit %u | ROI x%d y%d %dx%d\n",
                gK.ok ? "ADA" : "BELUM", gK.floorMean, gK.floorStd, gK.ok ? ambangGelapKalib() : 0, normalTotal(), gK.sdDark,
                batasTotal(), rX0, rY0, rW, rH);
  KONSOL.printf("# param: periode %u ms, k %.1f, delta %d, rasio %.2f, hist %d/%d, batasManual %d, gelapManual %d, kompensasi %d, LED %d\n",
                gP.periodeMs, gP.kGelapX10 / 10.0f, gP.deltaGelap, gP.rasioBatasX100 / 100.0f, gP.histNyala, gP.histMati,
                gP.batasManual, gP.gelapManual, gP.kompensasi, gP.ledFlash);
}

static void prosesTeks(char *s) {
  char *cmd = strtok(s, " \t");
  if (!cmd) return;
  char *a1 = strtok(nullptr, " \t");
  char *a2 = strtok(nullptr, " \t");
  if (!strcmp(cmd, "bantuan") || !strcmp(cmd, "?")) bantuan();
  else if (!strcmp(cmd, "status")) cetakStatus();
  else if (!strcmp(cmd, "mulai")) { gDeteksi = gKameraOk; KONSOL.println(gDeteksi ? F("# deteksi mulai") : F("# kamera gagal")); }
  else if (!strcmp(cmd, "berhenti")) { gDeteksi = false; KONSOL.println(F("# deteksi berhenti")); }
  else if (!strcmp(cmd, "kalib")) { if (gKameraOk && !gKalibFase) mulaiKalibrasi(); }
  else if (!strcmp(cmd, "hapus")) { gK = Kalibrasi(); simpanKalib(); KONSOL.println(F("# kalibrasi dihapus")); }
  else if (!strcmp(cmd, "debug") && a1) { gDebug = atoi(a1) != 0; }
  else if (!strcmp(cmd, "led") && a1) { setParam(PKP_LED_FLASH, atoi(a1) ? 1 : 0); }
  else if (!strcmp(cmd, "set") && a1 && a2) KONSOL.println(setParam((uint8_t)atoi(a1), (int16_t)atoi(a2)) ? F("# OK") : F("# nilai/id salah"));
  else if (!strcmp(cmd, "ascii")) cetakAscii();
  else KONSOL.println(F("# perintah tidak dikenal, ketik bantuan"));
}

static void prosesByte(PkPengurai &pg, uint8_t c, bool bolehTeks) {
  int r = pg.masukkan(c);
  if (r == 1) { prosesBingkai(pg); gTeksLen = 0; return; }
  if (r == -1 && bolehTeks) {
    if (c == '\n' || c == '\r') {
      if (gTeksLen) { gTeks[gTeksLen] = 0; prosesTeks(gTeks); gTeksLen = 0; }
    } else if (c >= 0x20 && c < 0x7F && gTeksLen < sizeof(gTeks) - 1) {
      gTeks[gTeksLen++] = (char)c;
    }
  }
}

static void bacaMasukan() {
  int budget = 256;
  while (LINK.available() && budget--) prosesByte(gPgLink, (uint8_t)LINK.read(), !KONSOL_TERPISAH);
#if KONSOL_TERPISAH
  budget = 128;
  while (KONSOL.available() && budget--) prosesByte(gPgKonsol, (uint8_t)KONSOL.read(), true);
#endif
}

// =====================================================================
void setup() {
#if PIN_LED_FLASH >= 0
  pinMode(PIN_LED_FLASH, OUTPUT);
  digitalWrite(PIN_LED_FLASH, LOW);   // LED flash MATI sejak awal
#endif
  LINK.setRxBufferSize(512);
  LINK.begin(PK_BAUD, SERIAL_8N1, LINK_PIN_RX, LINK_PIN_TX);
#if KONSOL_TERPISAH
  KONSOL.begin(115200);
#endif
  delay(100);
  KONSOL.printf("\n# visual-esp32cam v%s, papan %s, protokol v%d\n", FW_VERSI, PAPAN_NAMA, PK_VERSI);
  muatNvs();
  setLed(gP.ledFlash);
  gKameraOk = mulaiKamera();
  hitungRoi(REF_W, REF_H);
  if (!gKameraOk) kirimGalat(PKG_INIT_KAMERA, 0);
  KONSOL.printf("# kalibrasi lantai: %s. Ketik 'bantuan'.\n", gK.ok ? "dimuat dari NVS" : "BELUM ADA (jalankan 'kalib' di lantai bersih)");
  kirimHeartbeat();
}

void loop() {
  static uint32_t tHb = 0, tFrame = 0, tCobaUlang = 0;
  uint32_t now = millis();
  bacaMasukan();

  if (now - tHb >= HEARTBEAT_MS) { tHb = now; kirimHeartbeat(); }
  if (now - gTFps >= 1000) { gFpsX10 = (uint16_t)(gFrameDetik * 10000UL / (now - gTFps)); gFrameDetik = 0; gTFps = now; }

  // kamera gagal: coba init ulang tiap 5 s
  if (!gKameraOk) {
    if (now - tCobaUlang > 5000) {
      tCobaUlang = now;
      esp_camera_deinit();
      gKameraOk = mulaiKamera();
      if (!gKameraOk) kirimGalat(PKG_INIT_KAMERA, 1);
    }
    return;
  }

  bool perluFrame = gKalibFase ? true : gDeteksi;
  uint32_t periode = gKalibFase ? 100 : gP.periodeMs;
  if (!perluFrame || now - tFrame < periode) return;
  tFrame = now;

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    if (gErrFrame < 255) gErrFrame++;
    if (++gGagalBeruntun >= 5) {
      KONSOL.println(F("# 5 frame gagal berturut-turut, init ulang kamera"));
      kirimGalat(PKG_FRAME, gGagalBeruntun);
      gGagalBeruntun = 0;
      gKameraOk = false;
      tCobaUlang = 0;
    }
    return;
  }
  gGagalBeruntun = 0;
  hitungRoi(fb->width, fb->height);
  gFrameDetik++;
  if (gKalibFase) langkahKalibrasi(fb);
  else deteksi(fb);
  esp_camera_fb_return(fb);
}
