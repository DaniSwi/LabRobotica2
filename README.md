# Laboratorio 2 — Navegación Reactiva con Filtrado y Fusión de Sensores en Webots

**Curso:** Robótica y Sistemas Autónomos 2026-01 · ICI 4150  
**Integrantes:** [Daniel Cornejo] · [Ian Guerrero] · [Isidora Osorio]

---

## Objetivo

Implementar un sistema de navegación reactiva en Webots para el robot e-puck utilizando sensores de proximidad y encoders de rueda, aplicando un filtro de media móvil sobre las mediciones crudas y un filtro de Kalman para estimar la distancia frontal al obstáculo más cercano, mejorando así la robustez de las decisiones de movimiento.

---

## Robot y sensores utilizados

| Elemento | Descripción |
|---|---|
| Robot | e-puck (diferencial, 2 ruedas) |
| Radio de rueda | r = 0.0205 m |
| Distancia entre ejes | L = 0.052 m |
| Sensores de proximidad | 8 × IR (ps0–ps7), valor crudo 0–4095 |
| Encoders | `left wheel sensor` y `right wheel sensor` (rad acumulados) |

**Distribución de sensores relevantes:**

```
         ps7  ps0
      ps6        ps1
    ps5            ps2
      ps4        ps3
```

- **Frontales:** ps0 (frontal derecho) y ps7 (frontal izquierdo)  
- **Laterales:** ps1 (lateral derecho) y ps6 (lateral izquierdo)  
- **Encoders:** posición angular acumulada en radianes

---

## Frecuencia de muestreo

| Parámetro | Valor |
|---|---|
| Paso de simulación (TIME_STEP) | 64 ms |
| Tiempo de muestreo Ts | 0.064 s |
| Frecuencia de muestreo fs | ≈ 15.6 Hz |

Todas las señales (crudas, filtradas, estimadas) se registran a esta misma frecuencia.

---

## Estructura del repositorio

```
lab2/
├── controllers/
│   └── epuck_controller/
│       └── epuck_controller.py   ← Controlador principal
├── worlds/
│   ├── escenario_simple.wbt
│   └── escenario_complejo.wbt
├── plot_signals.py               ← Script de análisis y gráficos
├── sensor_log.csv                ← Log generado tras simulación
└── README.md
```

---

## Descripción de la solución implementada

### 1. Conversión sensor → distancia (cm)

Los sensores de proximidad del e-puck entregan valores crudos en el rango `[0, 4095]`. Se aplica la relación inversa:

```
d(cm) = k / raw     donde k ≈ 900
```

Los valores fuera del rango razonable se recortan a `[1, 100]` cm.

---

### 2. Estimación de avance mediante encoders

Los encoders devuelven la posición angular acumulada θ (rad) de cada rueda. El desplazamiento lineal entre dos instantes se calcula como:

```
Δd_k = r · (Δθ_izq + Δθ_der) / 2
```

donde `r = 0.0205 m` es el radio de rueda. Esto entrega la predicción de cuánto se acercó el robot al obstáculo frontal.

---

### 3. Filtro simple: Media Móvil

Se aplica un filtro de media móvil de ventana `N = 5` sobre la distancia frontal cruda:

```
d̄_k = (1/N) · Σ d_raw_{k-N+1 .. k}
```

Este filtro atenúa el ruido de alta frecuencia manteniendo un retardo mínimo, y sirve como entrada a la etapa de corrección del filtro de Kalman.

---

### 4. Filtro de Kalman — Estimación de distancia frontal

El estado del filtro es la distancia frontal estimada `d̂_k` (en cm).

#### Etapa de predicción

A partir del avance estimado con encoders:

```
d̂⁻_k = d̂_{k-1} − Δd_k
P⁻_k  = P_{k-1} + Q
```

El robot avanza → la distancia al obstáculo disminuye, de ahí el signo negativo.

#### Etapa de corrección

Con la lectura filtrada de los sensores frontales `z_k`:

```
K_k   = P⁻_k / (P⁻_k + R)
d̂_k  = d̂⁻_k + K_k · (z_k − d̂⁻_k)
P_k   = (1 − K_k) · P⁻_k
```

#### Parámetros utilizados

| Parámetro | Valor | Descripción |
|---|---|---|
| Q | 1×10⁻³ | Ruido de proceso (modelo cinemático) |
| R | 5.0 cm² | Varianza de medición sensor frontal |
| d̂₀ | 100.0 cm | Distancia inicial asumida |

**Interpretación de la ganancia K_k:**
- R grande → K pequeño → mayor confianza en la predicción (encoders)  
- P⁻_k grande → K grande → mayor confianza en la medición (sensores)

---

### 5. Lógica de navegación reactiva

```python
if d̂_k (Kalman) > 12.0 cm:
    → AVANZAR  (vl = vr = 70 % MAX_SPEED)
else:
    if obstáculo_derecha > obstáculo_izquierda:
        → GIRAR IZQUIERDA
    else:
        → GIRAR DERECHA
```

Los sensores laterales `ps1` y `ps6` (sumados con `ps2` y `ps5` respectivamente) determinan hacia qué lado hay más obstáculo y así el robot siempre escoge el lado más libre.

---

## Escenarios de prueba

### Escenario 1 — Simple (pocos obstáculos)
- Arena rectangular con 2–3 cajas dispersas.  
- Se evalúa: convergencia del filtro Kalman, pocos giros, avance fluido.

### Escenario 2 — Complejo (pasillos y obstáculos múltiples)
- Laberinto en U con paredes laterales cercanas.  
- Se evalúa: capacidad de navegar pasillos estrechos, estabilidad del estimador bajo ruido lateral intenso.

---

## Señales registradas (sensor_log.csv)

| Columna | Descripción |
|---|---|
| `time_s` | Tiempo de simulación (s) |
| `ps0_raw`, `ps7_raw` | Sensores frontales crudos |
| `ps1_raw`, `ps6_raw` | Sensores laterales crudos |
| `enc_left_rad`, `enc_right_rad` | Encoders (rad acumulados) |
| `delta_d_cm` | Avance estimado Δd (cm) |
| `front_raw_cm` | Distancia frontal cruda (cm) |
| `front_filtered_cm` | Distancia filtrada (MA, cm) |
| `front_kalman_cm` | Distancia estimada Kalman (cm) |
| `kalman_gain` | Ganancia K_k |
| `action` | Acción tomada |

---

## Gráficos generados

Ejecutar `plot_signals.py` produce tres figuras:

- **fig1_distancias_frontales.png** — Comparación cruda / filtrada / Kalman  
- **fig2_sensores_encoders.png** — Señales crudas laterales + encoders + Δd  
- **fig3_kalman_gain.png** — Evolución temporal de la ganancia K_k

---

## Análisis y conclusiones

- La señal cruda presenta picos abruptos de ruido. El filtro de media móvil los atenúa pero introduce un leve retardo de N/2 pasos.  
- El filtro de Kalman, al combinar la predicción cinemática (encoders) con la medición filtrada, produce una estimación suave y con menor latencia efectiva que el filtro MA solo.  
- La ganancia K_k converge rápidamente en los primeros ciclos y se estabiliza en un valor que refleja el equilibrio entre Q y R.  
- El robot navega con mayor estabilidad usando la distancia Kalman que con lecturas crudas: se reducen los giros innecesarios causados por picos de ruido.

---

## Instrucciones para ejecutar la simulación

### Requisitos
- **Webots** R2023b o superior  
- **Python** 3.9+  
- `matplotlib` (solo para gráficos): `pip install matplotlib`

### Pasos

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/DaniSwi/LabRobotica2.git
   cd LabRobotica2
   ```

2. Abrir Webots y cargar el mundo deseado:
   ```
   File → Open World → worlds/escenario_simple.wbt
   ```

3. Verificar que el controlador del robot apunta a `epuck_controller`.

4. Ejecutar la simulación (botón ▶). El archivo `sensor_log.csv` se genera automáticamente en el directorio del controlador.

5. Analizar las señales:
   ```bash
   python plot_signals.py controllers/epuck_controller/sensor_log.csv
   ```

---

*Laboratorio 2 — ICI 4150 · 2026-01*
