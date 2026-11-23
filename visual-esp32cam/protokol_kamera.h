// =====================================================================
//  protokol_kamera.h - Protokol UART ESP32-S3 (utama) <-> modul kamera
//  Spesifikasi lengkap: 06 Implementasi\07 Integrasi\Protokol_UART_Kamera.md
//
//  BERKAS INI DIPAKAI BERSAMA oleh main-esp32s3 dan visual-esp32cam.
//  Kedua salinan HARUS identik. Bila mengubah, ubah keduanya dan naikkan PK_VERSI.
//
//  Bingkai (frame), semua bilangan multi-byte little-endian:
//    [SOF 0xA5][TIPE][SEQ][LEN][PAYLOAD 0..32 byte][CRC8]
//    CRC8 = CRC-8/SMBUS (poli 0x07, awal 0x00, tanpa refleksi, tanpa XOR akhir)
//           dihitung atas TIPE, SEQ, LEN, PAYLOAD (SOF tidak ikut).
//  SOF 0xA5 sengaja > 0x7F: teks ASCII (log boot ROM, debug) tidak pernah
//  memuat byte ini, sehingga teks yang bercampur di jalur yang sama diabaikan.
// =====================================================================
#pragma once
#include <stdint.h>
#include <stddef.h>

#define PK_VERSI          1
#define PK_SOF            0xA5
#define PK_MAKS_PAYLOAD   32
#define PK_MAKS_BINGKAI   (PK_MAKS_PAYLOAD + 5)
#define PK_BAUD           115200

// ---------------- Tipe pesan ----------------
// Utama -> kamera
#define PK_PING           0x01  // len 0            -> balas HEARTBEAT segera
#define PK_MULAI          0x02  // len 0 atau 2     (u16 periode_ms, opsional) -> ACK
#define PK_BERHENTI       0x03  // len 0            -> ACK
#define PK_MINTA_STATUS   0x04  // len 0            -> STATUS
#define PK_SET_PARAM      0x05  // len 3            (u8 id, i16 nilai) -> ACK
#define PK_KALIBRASI      0x06  // len 0            -> ACK, lalu STATUS saat selesai (atau GALAT)
#define PK_HAPUS_KALIB    0x07  // len 0            -> ACK
// Kamera -> utama
#define PK_HEARTBEAT      0x81  // len 8, periodik 1 Hz (juga balasan PING)
#define PK_STATUS         0x82  // len 12
#define PK_KOTOR          0x83  // len 12, hasil deteksi tiap periode (bawaan 150 ms)
#define PK_ACK            0x8F  // len 3
#define PK_GALAT          0xEE  // len 2

// ---------------- Bit flag status kamera (HEARTBEAT/STATUS byte 0) ----------------
#define PKF_DETEKSI_AKTIF   0x01
#define PKF_TERKALIBRASI    0x02
#define PKF_KALIBRASI_JALAN 0x04
#define PKF_KAMERA_OK       0x08
#define PKF_LED_NYALA       0x10

// ---------------- Bit flag hasil KOTOR (byte 1) ----------------
#define PKK_KOTOR           0x01  // keputusan setelah histeresis
#define PKK_KANDIDAT        0x02  // frame ini melewati batas (sebelum histeresis)
#define PKK_TERKALIBRASI    0x04  // ambang berasal dari kalibrasi lantai bersih
#define PKK_CAHAYA_ANEH     0x08  // rata-rata ROI menyimpang jauh dari lantai kalibrasi (silau/gelap)

// ---------------- Bit region (byte 2 KOTOR) ----------------
#define PKR_KIRI            0x01
#define PKR_TENGAH          0x02
#define PKR_KANAN           0x04

// ---------------- ID parameter SET_PARAM ----------------
#define PKP_PERIODE_MS      0x01  // 50..1000, bawaan 150
#define PKP_K_GELAP_X10     0x02  // k-sigma ambang gelap x10, 5..100, bawaan 30
#define PKP_DELTA_GELAP     0x03  // selisih minimum lantai - ambang gelap (level abu), 5..120, bawaan 20
#define PKP_RASIO_BATAS_X100 0x04 // batas kotor = normal x rasio/100, 105..400, bawaan 145
#define PKP_HIST_NYALA      0x05  // frame berturut-turut untuk KOTOR=1, 1..20, bawaan 3
#define PKP_HIST_MATI       0x06  // frame berturut-turut untuk KOTOR=0, 1..20, bawaan 3
#define PKP_BATAS_MANUAL    0x07  // batas kotor absolut (piksel), 0 = otomatis
#define PKP_GELAP_MANUAL    0x08  // ambang gelap absolut (0..255), 0 = otomatis
#define PKP_KOMPENSASI      0x09  // 1 = ambang mengikuti rata-rata kecerahan frame, 0 = tetap
#define PKP_LED_FLASH       0x0A  // 0 = mati (bawaan, cegah silau), 1 = nyala

// ---------------- Status ACK ----------------
#define PKA_OK              0
#define PKA_TIDAK_DIKENAL   1
#define PKA_NILAI_SALAH     2
#define PKA_SIBUK           3
#define PKA_KAMERA_GAGAL    4

// ---------------- Kode GALAT ----------------
#define PKG_INIT_KAMERA     1
#define PKG_FRAME           2
#define PKG_KALIBRASI       3
#define PKG_BINGKAI         4

// ---------------- ID papan kamera (STATUS byte 11) ----------------
#define PK_PAPAN_AITHINKER  1
#define PK_PAPAN_S3_EYE     2
#define PK_PAPAN_KUSTOM     3

// =====================================================================
//  Fungsi bantu (header-only)
// =====================================================================
static inline uint8_t pkCrc8(const uint8_t *d, size_t n, uint8_t crc = 0x00) {
  for (size_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

static inline void pkTulisU16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v & 0xFF); p[1] = (uint8_t)(v >> 8); }
static inline uint16_t pkBacaU16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

// Susun bingkai ke 'out' (minimal PK_MAKS_BINGKAI byte). Kembalikan panjang total, 0 bila len terlalu besar.
static inline size_t pkBangun(uint8_t *out, uint8_t tipe, uint8_t seq, const uint8_t *payload, uint8_t len) {
  if (len > PK_MAKS_PAYLOAD) return 0;
  out[0] = PK_SOF;
  out[1] = tipe;
  out[2] = seq;
  out[3] = len;
  for (uint8_t i = 0; i < len; i++) out[4 + i] = payload[i];
  out[4 + len] = pkCrc8(out + 1, 3 + len);
  return 5 + len;
}

// Pengurai bingkai byte demi byte. masukkan() mengembalikan:
//   1  = bingkai lengkap dan CRC benar (data di tipe/seq/len/payload)
//   0  = byte dipakai / sedang mengumpulkan bingkai
//  -1  = byte bukan bagian bingkai (di luar bingkai dan bukan SOF), boleh dipakai sebagai teks
struct PkPengurai {
  uint8_t st = 0;
  uint8_t tipe = 0, seq = 0, len = 0, idx = 0;
  uint8_t payload[PK_MAKS_PAYLOAD];
  uint32_t nOk = 0, nCrcSalah = 0, nPanjangSalah = 0;
  uint32_t tMulaiMs = 0;   // diisi pemanggil bila ingin batas waktu antar-byte

  void reset() { st = 0; idx = 0; }

  int masukkan(uint8_t c) {
    switch (st) {
      case 0:
        if (c == PK_SOF) { st = 1; return 0; }
        return -1;
      case 1: tipe = c; st = 2; return 0;
      case 2: seq = c; st = 3; return 0;
      case 3:
        len = c;
        if (len > PK_MAKS_PAYLOAD) { nPanjangSalah++; st = (c == PK_SOF) ? 1 : 0; return 0; }
        idx = 0;
        st = (len == 0) ? 5 : 4;
        return 0;
      case 4:
        payload[idx++] = c;
        if (idx >= len) st = 5;
        return 0;
      case 5: {
        uint8_t h[3] = {tipe, seq, len};
        uint8_t crc = pkCrc8(h, 3);
        crc = pkCrc8(payload, len, crc);
        st = 0;
        if (crc == c) { nOk++; return 1; }
        nCrcSalah++;
        return 0;
      }
    }
    st = 0;
    return 0;
  }
};
