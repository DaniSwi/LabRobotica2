"""
Laboratorio 2 - ICI 4150
Análisis y graficación de señales registradas por el controlador e-puck.
Ejecutar DESPUÉS de la simulación cuando sensor_log.csv esté disponible.

Uso:
    python plot_signals.py [ruta_al_csv]
"""

import sys
import csv
import math
import os

# ── Intentar importar matplotlib; si no está, instrucciones ──────────────────
try:
    import matplotlib
    matplotlib.use("Agg")          # sin ventana emergente (útil en Webots)
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec
    HAS_MPL = True
except ImportError:
    HAS_MPL = False
    print("[AVISO] matplotlib no encontrado. Instala con: pip install matplotlib")


# ─────────────────────────────────────────────
#  CARGA DE DATOS
# ─────────────────────────────────────────────
def load_csv(path: str) -> dict:
    """Carga el CSV de log y devuelve un dict de listas numéricas."""
    data = {}
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            for k, v in row.items():
                if k not in data:
                    data[k] = []
                try:
                    data[k].append(float(v))
                except ValueError:
                    data[k].append(v)   # strings (action)
    print(f"[Plot] {len(data.get('time_s', []))} muestras cargadas desde '{path}'")
    return data


# ─────────────────────────────────────────────
#  ESTADÍSTICAS BÁSICAS
# ─────────────────────────────────────────────
def stats(values: list) -> dict:
    n    = len(values)
    mean = sum(values) / n
    var  = sum((x - mean) ** 2 for x in values) / n
    return {
        "n"   : n,
        "mean": round(mean, 4),
        "std" : round(math.sqrt(var), 4),
        "min" : round(min(values), 4),
        "max" : round(max(values), 4),
    }


def print_stats(data: dict):
    print("\n══════════════════════════════════════════")
    print("  ESTADÍSTICAS DE SEÑALES")
    print("══════════════════════════════════════════")
    cols = ["front_raw_cm", "front_filtered_cm", "front_kalman_cm",
            "delta_d_cm", "kalman_gain"]
    for col in cols:
        if col in data:
            s = stats(data[col])
            print(f"  {col:<22}: mean={s['mean']:7.3f}  std={s['std']:7.3f}  "
                  f"min={s['min']:7.3f}  max={s['max']:7.3f}")

    # Frecuencia de muestreo estimada
    t = data.get("time_s", [])
    if len(t) > 1:
        Ts_est = (t[-1] - t[0]) / (len(t) - 1)
        fs_est = 1.0 / Ts_est if Ts_est > 0 else 0
        print(f"\n  Ts estimado : {Ts_est*1000:.2f} ms")
        print(f"  fs estimada : {fs_est:.2f} Hz")
        print(f"  N muestras  : {len(t)}")

    # Conteo de acciones
    actions = data.get("action", [])
    for act in ["avanzar", "girar_der", "girar_izq"]:
        count = actions.count(act)
        pct   = 100 * count / len(actions) if actions else 0
        print(f"  {act:<12}: {count:5d} pasos ({pct:.1f}%)")
    print("══════════════════════════════════════════\n")


# ─────────────────────────────────────────────
#  GRAFICACIÓN
# ─────────────────────────────────────────────
def plot_signals(data: dict, out_dir: str = "."):
    if not HAS_MPL:
        return

    t   = data["time_s"]
    raw = data["front_raw_cm"]
    flt = data["front_filtered_cm"]
    kal = data["front_kalman_cm"]
    kg  = data["kalman_gain"]

    ps0 = data["ps0_raw"]
    ps7 = data["ps7_raw"]
    ps1 = data["ps1_raw"]
    ps6 = data["ps6_raw"]

    enc_l = data["enc_left_rad"]
    enc_r = data["enc_right_rad"]
    dd    = data["delta_d_cm"]

    # ── Figura 1: Distancias frontales ──────────────────────────
    fig1, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    fig1.suptitle("Laboratorio 2 — Señales de Distancia Frontal", fontsize=14, fontweight="bold")

    axes[0].plot(t, raw, color="#e74c3c", linewidth=0.8, label="Cruda")
    axes[0].set_ylabel("Distancia (cm)")
    axes[0].set_title("Señal cruda del sensor frontal")
    axes[0].legend(); axes[0].grid(alpha=0.3)

    axes[1].plot(t, raw, color="#e74c3c", linewidth=0.6, alpha=0.5, label="Cruda")
    axes[1].plot(t, flt, color="#2ecc71", linewidth=1.2, label="Filtrada (MA)")
    axes[1].set_ylabel("Distancia (cm)")
    axes[1].set_title("Señal filtrada (media móvil)")
    axes[1].legend(); axes[1].grid(alpha=0.3)

    axes[2].plot(t, flt, color="#2ecc71", linewidth=0.8, alpha=0.6, label="Filtrada")
    axes[2].plot(t, kal, color="#3498db", linewidth=1.4, label="Kalman")
    axes[2].axhline(y=12.0, color="#e67e22", linestyle="--", linewidth=1,
                    label="Umbral seguridad (12 cm)")
    axes[2].set_ylabel("Distancia (cm)")
    axes[2].set_xlabel("Tiempo (s)")
    axes[2].set_title("Estimación con filtro de Kalman")
    axes[2].legend(); axes[2].grid(alpha=0.3)

    plt.tight_layout()
    p1 = os.path.join(out_dir, "fig1_distancias_frontales.png")
    fig1.savefig(p1, dpi=150, bbox_inches="tight")
    print(f"[Plot] Guardado: {p1}")
    plt.close(fig1)

    # ── Figura 2: Sensores crudos y encoders ────────────────────
    fig2 = plt.figure(figsize=(12, 9))
    fig2.suptitle("Laboratorio 2 — Sensores Crudos y Encoders", fontsize=14, fontweight="bold")
    gs = gridspec.GridSpec(3, 2, figure=fig2)

    ax_f = fig2.add_subplot(gs[0, :])
    ax_f.plot(t, ps0, label="ps0 (frontal der)", color="#e74c3c")
    ax_f.plot(t, ps7, label="ps7 (frontal izq)", color="#e67e22")
    ax_f.set_ylabel("Valor crudo")
    ax_f.set_title("Sensores frontales (ps0, ps7)")
    ax_f.legend(); ax_f.grid(alpha=0.3)

    ax_l = fig2.add_subplot(gs[1, 0])
    ax_l.plot(t, ps6, color="#9b59b6", label="ps6 (lat. izq)")
    ax_l.set_ylabel("Valor crudo"); ax_l.set_title("Sensor lateral izquierdo")
    ax_l.legend(); ax_l.grid(alpha=0.3)

    ax_r = fig2.add_subplot(gs[1, 1])
    ax_r.plot(t, ps1, color="#1abc9c", label="ps1 (lat. der)")
    ax_r.set_ylabel("Valor crudo"); ax_r.set_title("Sensor lateral derecho")
    ax_r.legend(); ax_r.grid(alpha=0.3)

    ax_enc = fig2.add_subplot(gs[2, 0])
    ax_enc.plot(t, enc_l, color="#2980b9", label="Encoder izq (rad)")
    ax_enc.plot(t, enc_r, color="#c0392b", label="Encoder der (rad)")
    ax_enc.set_xlabel("Tiempo (s)"); ax_enc.set_ylabel("Posición angular (rad)")
    ax_enc.set_title("Encoders de rueda")
    ax_enc.legend(); ax_enc.grid(alpha=0.3)

    ax_dd = fig2.add_subplot(gs[2, 1])
    ax_dd.plot(t, dd, color="#27ae60", linewidth=0.9)
    ax_dd.set_xlabel("Tiempo (s)"); ax_dd.set_ylabel("Δd (cm)")
    ax_dd.set_title("Avance estimado por encoders (Δd_k)")
    ax_dd.grid(alpha=0.3)

    plt.tight_layout()
    p2 = os.path.join(out_dir, "fig2_sensores_encoders.png")
    fig2.savefig(p2, dpi=150, bbox_inches="tight")
    print(f"[Plot] Guardado: {p2}")
    plt.close(fig2)

    # ── Figura 3: Ganancia de Kalman ────────────────────────────
    fig3, ax3 = plt.subplots(figsize=(12, 4))
    fig3.suptitle("Laboratorio 2 — Ganancia de Kalman", fontsize=14, fontweight="bold")
    ax3.plot(t, kg, color="#8e44ad", linewidth=1.0)
    ax3.set_xlabel("Tiempo (s)"); ax3.set_ylabel("K_k")
    ax3.set_title("Evolución de la ganancia de Kalman")
    ax3.grid(alpha=0.3)
    plt.tight_layout()
    p3 = os.path.join(out_dir, "fig3_kalman_gain.png")
    fig3.savefig(p3, dpi=150, bbox_inches="tight")
    print(f"[Plot] Guardado: {p3}")
    plt.close(fig3)

    print("[Plot] Todas las figuras generadas correctamente.")


# ─────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────
if __name__ == "__main__":
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "sensor_log.csv"

    if not os.path.exists(csv_path):
        print(f"[Error] No se encontró el archivo: {csv_path}")
        print("  Ejecuta primero la simulación en Webots para generar el log.")
        sys.exit(1)

    data    = load_csv(csv_path)
    out_dir = os.path.dirname(os.path.abspath(csv_path))

    print_stats(data)
    plot_signals(data, out_dir=out_dir)
