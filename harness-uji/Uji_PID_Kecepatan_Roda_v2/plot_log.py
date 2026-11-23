"""Plot log CSV dari Uji_PID_Kecepatan_Roda_v2 dan hitung metrik respons step.

Cara pakai:
  1. Di Serial Monitor (115200), kirim "a 100" (tanpa pembatas akselerasi) lalu "S" (uji step),
     atau "R" untuk uji ramp. Salin semua baris CSV ke file, mis. log_step.csv.
  2. python plot_log.py log_step.csv
Keluaran: log_step.png dan tabel metrik (rise time 10-90%, overshoot, settling time 2%, error steady-state).
"""
import sys
import csv
import numpy as np
import matplotlib.pyplot as plt

M_PER_TICK = 2 * np.pi * 0.0335 / 517     # samakan dengan CPR_1X di sketch
TS = 0.05


def baca(path):
    rows = []
    with open(path, encoding="utf-8", errors="ignore") as f:
        for r in csv.reader(f):
            if len(r) == 10 and r[0].strip().isdigit():
                rows.append([float(x) for x in r])
    return np.array(rows)


def metrik(t, ref, v):
    """Metrik untuk tiap segmen dengan referensi konstan."""
    hasil = []
    batas = np.flatnonzero(np.abs(np.diff(ref)) > 1e-6) + 1
    seg = np.split(np.arange(len(t)), batas)
    for k in range(1, len(seg)):
        i = seg[k]
        r0, r1 = ref[seg[k - 1][-1]], ref[i[0]]
        if abs(r1 - r0) < 1 or len(i) < 5:
            continue
        y = v[i]
        tt = t[i] - t[i[0]]
        step = r1 - r0
        frac = (y - r0) / step
        try:
            tr = tt[np.argmax(frac >= 0.9)] - tt[np.argmax(frac >= 0.1)]
        except ValueError:
            tr = np.nan
        os_ = max(0.0, (np.max(frac) - 1) * 100)
        luar = np.flatnonzero(np.abs(y - r1) > max(0.02 * abs(step), 0.5))   # 2% atau resolusi 0,5 tick
        ts = tt[luar[-1]] + TS if len(luar) else 0.0
        ess = np.mean(y[-max(5, len(y) // 4):]) - r1
        hasil.append((r0, r1, tr, os_, ts, ess))
    return hasil


def main(path):
    d = baca(path)
    t = (d[:, 0] - d[0, 0]) / 1000
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for col_r, col_v, nama in ((1, 3, "kiri"), (2, 4, "kanan")):
        ax[0].plot(t, d[:, col_r], "--", label=f"referensi {nama}")
        ax[0].plot(t, d[:, col_v], label=f"terukur {nama}")
    ax[0].set_ylabel("kecepatan [tick 1x / 50 ms]")
    sec = ax[0].secondary_yaxis("right", functions=(lambda x: x * M_PER_TICK / TS, lambda y: y * TS / M_PER_TICK))
    sec.set_ylabel("[m/s] (CPR asumsi 517)")
    ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].plot(t, d[:, 5], label="PWM kiri"); ax[1].plot(t, d[:, 6], label="PWM kanan")
    ax[1].set_ylabel("PWM [-255..255]"); ax[1].set_xlabel("waktu [s]"); ax[1].legend(); ax[1].grid(alpha=.3)
    ax[0].set_title("Respons kecepatan roda (data terukur)")
    out = path.rsplit(".", 1)[0] + ".png"
    fig.tight_layout(); fig.savefig(out, dpi=150)
    print("grafik:", out)
    for col_r, col_v, nama in ((1, 3, "kiri"), (2, 4, "kanan")):
        print(f"\nRoda {nama}: ref_awal -> ref_akhir | rise 10-90% [s] | overshoot [%] | settling 2% [s] | e_ss [tick]")
        for r0, r1, tr, os_, ts, ess in metrik(t, d[:, col_r], d[:, col_v]):
            print(f"  {r0:6.1f} -> {r1:6.1f} | {tr:5.2f} | {os_:5.1f} | {ts:5.2f} | {ess:+.2f}")
    print(f"\nOdometri akhir: x={d[-1,7]:.3f} m, y={d[-1,8]:.3f} m, theta={d[-1,9]:.1f} deg")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "log_step.csv")
