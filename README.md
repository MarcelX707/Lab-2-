# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión Sensorial

**Asignatura:** Robótica y Sistemas Autónomos 2026-01  
**Código:** ICI 4150  
**Institución:** Pontificia Universidad Católica de Valparaíso (PUCV)

---

## Integrantes del Grupo
* Mario Rojas
* Marcel Gutierrez
* Benjamín Leiva
* Martín Basulto

---

## Objetivo

Implementar un sistema de navegación reactiva en Webots para un robot móvil diferencial, utilizando sensores de distancia y encoders de rueda. Se aplica un filtro exponencial de paso bajo sobre las mediciones crudas y un Filtro de Kalman para estimar la distancia frontal a obstáculos, mejorando la toma de decisiones autónoma del robot.

---

## 1. Descripción del Robot y Sensores

Se utilizó el robot **e-puck** de Webots, un robot diferencial de dos ruedas motrices independientes. La configuración de sensores empleada fue la siguiente:

| Sensor | Dispositivo | Uso |
|--------|------------|-----|
| Frontal izquierdo | `ps7` | Fusión sensorial (medición Kalman) |
| Frontal derecho | `ps0` | Fusión sensorial (medición Kalman) |
| Diagonal izquierdo | `ps6` | Apoyo en detección de esquinas (peso 0.7) |
| Diagonal derecho | `ps1` | Apoyo en detección de esquinas (peso 0.7) |
| Lateral izquierdo | `ps5` | Decisión de dirección de giro |
| Lateral derecho | `ps2` | Decisión de dirección de giro |
| Encoder izquierdo | `left wheel sensor` | Odometría / predicción Kalman |
| Encoder derecho | `right wheel sensor` | Odometría / predicción Kalman |

El radio de la rueda del e-puck utilizado es **r = 0.0205 m**.

---

## 2. Frecuencia de Muestreo

Los parámetros de muestreo utilizados en la simulación son:

| Parámetro | Valor |
|-----------|-------|
| Tiempo de muestreo (Tₛ) | **0.032 s** |
| Frecuencia de muestreo (fₛ) | **31.25 Hz** |
| Muestras escenario 1 | **322** |
| Muestras escenario 2 | **15 803** |

Todas las señales registradas —crudas, filtradas y estimadas— se analizan bajo esta misma base temporal.

---

## 3. Análisis de Señales Registradas

### 3.1 Escenario 1 — Entorno Simple

Duración: **10.27 s** | Muestras: **322**

| Señal | Mínimo | Máximo | Media |
|-------|--------|--------|-------|
| Raw | 65.16 | 119.76 | 82.58 |
| Filtrado exponencial | 20.92 | 114.53 | 82.06 |
| Kalman | 22.68 | 112.86 | 81.92 |

La señal cruda presenta fluctuaciones visibles entre muestras consecutivas. El filtro exponencial suaviza estas variaciones con una constante α = 0.2, convergiendo progresivamente hacia el valor real. El estimador de Kalman presenta la curva más estable, con una ganancia promedio de **K ≈ 0.392**, lo que indica que el filtro confía principalmente en su predicción interna (coherente con Q = 0.05 y R = 0.8).

Aproximadamente el **35.4%** de las muestras superaron el umbral crítico de 80 unidades, lo que refleja la presencia del único obstáculo en el entorno.

### 3.2 Escenario 2 — Laberinto

Duración: **505.66 s** | Muestras: **15 803**

| Señal | Mínimo | Máximo | Media |
|-------|--------|--------|-------|
| Raw | 58.92 | 1048.32 | 71.65 |
| Filtrado exponencial | 53.41 | 852.56 | 71.65 |
| Kalman | 61.02 | 787.06 | 71.88 |

El laberinto expone al robot a múltiples obstáculos cercanos de forma continua. La señal cruda presenta picos abruptos de hasta 1048 unidades, mientras que el filtro de Kalman (con sintonía agresiva: Q = 2.0, R = 0.2) reacciona rápidamente manteniendo una estimación más acotada. La ganancia de Kalman se estabiliza en **K ≈ 0.916**, reflejando que el filtro confía principalmente en la medición del sensor (R pequeño) para responder con mayor velocidad ante obstáculos en pasillos estrechos. Se detectaron **60 eventos de giro** durante la simulación, ocupando el **4.0%** del tiempo total de navegación.

---

## 4. Estimación del Avance mediante Encoders (Odometría)

Los encoders de rueda del e-puck entregan el desplazamiento angular acumulado en **radianes**. El desplazamiento lineal en cada instante se calcula como:

```
s = r · θ
```

donde `r = 0.0205 m` es el radio de la rueda y `θ` es la variación angular entre dos pasos consecutivos. Para el avance conjunto del robot se promedia el desplazamiento de ambas ruedas:

```
Δs = r · (ΔθL + ΔθR) / 2
```

Esta estimación se convierte a las mismas unidades que los sensores de proximidad (×1000) para utilizarse como término de predicción en el filtro de Kalman:

```
avance_enc_uds = Δs × 1000
```

---

## 5. Filtro Simple Aplicado

Se implementó un **filtro exponencial de paso bajo** (EMA) sobre la señal frontal compuesta. La ecuación de actualización es:

```
z_k_filtrado = α · z_k_filtrado_anterior + (1 − α) · z_k_raw
```

Con **α = 0.2**, el filtro otorga mayor peso a la nueva medición (80%) y suaviza las variaciones bruscas. La señal de entrada (`z_k_raw`) no es solo el sensor frontal directo, sino el **máximo ponderado** del campo visual ampliado:

```
z_k_raw = max(ps7, ps0, ps6×0.7, ps1×0.7)
```

Esto permite detectar obstáculos que se aproximan tanto frontalmente como en diagonal, anticipando colisiones antes de que lleguen al campo frontal directo.

**Comparación con señal cruda:** en el escenario 1 la señal cruda varía abruptamente entre muestras (rango 65–120 uds), mientras que el filtrado exponencial suaviza estas transiciones con un ligero retardo de respuesta. En el laberinto, la diferencia es más pronunciada: los picos máximos de la señal cruda (1048 uds) se atenúan a 853 uds con el filtro exponencial.

---

## 6. Implementación del Filtro de Kalman

El filtro de Kalman estima la **distancia frontal al obstáculo más cercano** (`d̂_k`) combinando dos fuentes de información: la predicción basada en el movimiento del robot y la corrección basada en los sensores de distancia.

### 6.1 Etapa de Predicción

La distancia frontal estimada se actualiza restando el avance del robot al valor anterior. Si el robot avanza, se acerca al obstáculo; por lo tanto la proximidad aumenta:

```
d̂⁻_k = d̂_{k−1} + Δs_enc    (si avanzando recto)
d̂⁻_k = d̂_{k−1}              (si girando/escapando — Kalman congelado)
P⁻_k  = P_{k−1} + Q
```

**Kalman congelado:** durante giros, escapes y empujes, la predicción por encoder no es válida (el robot no avanza en línea recta), por lo que la etapa de predicción se omite y se mantiene el valor anterior. Esto evita divergencias en la estimación.

Parámetros de ruido de proceso:
- Escenario 1: **Q = 0.05** (proceso lento, poco ruido de predicción)
- Escenario 2: **Q = 2.0** (sintonía agresiva para respuesta rápida en laberinto)

### 6.2 Etapa de Corrección

La predicción se ajusta con la medición del sensor filtrado (`z_k_filtrado`):

```
K_k   = P⁻_k / (P⁻_k + R)
d̂_k  = d̂⁻_k + K_k · (z_k_filtrado − d̂⁻_k)
P_k   = (1 − K_k) · P⁻_k
```

La **Ganancia de Kalman** K_k pondera automáticamente ambas fuentes según su incertidumbre relativa:

- Si R es grande (sensor ruidoso): K_k pequeño → el filtro confía más en la predicción
- Si P⁻_k es grande (predicción incierta): K_k grande → el filtro confía más en el sensor

Parámetros de ruido de medición:
- Escenario 1: **R = 0.8** → K promedio = 0.392 (predomina la predicción)
- Escenario 2: **R = 0.2** → K promedio = 0.916 (predomina el sensor, respuesta más veloz)

---

## 7. Lógica de Navegación Reactiva

La navegación se implementó como una **máquina de estados de 4 estados**, utilizando `d̂_k` (distancia estimada por Kalman) como variable de decisión principal.

```
Estado 0: AVANCE CRUCERO
Estado 1: GIRO NORMAL
Estado 2: ESCAPE CIEGO
Estado 3: EMPUJE EXPLORATORIO
```

### Transiciones y umbrales

| Escenario | UMBRAL_CRITICO | UMBRAL_SALIDA | VEL_CRUCERO | VEL_GIRO |
|-----------|---------------|--------------|-------------|----------|
| Simple    | 80 uds        | 30 uds       | 5.0 rad/s   | 2.5 rad/s |
| Laberinto | 85 uds        | 75 uds       | 3.5 rad/s   | 2.0 rad/s |

### Descripción de los estados

**Estado 0 — Avance Crucero:** el robot mantiene `VEL_CRUCERO` mientras `d̂_k < UMBRAL_CRITICO`. Al superarse el umbral, se decide la dirección de giro comparando la presión lateral:

```python
presion_izq = ps5 + ps6
presion_der = ps2 + ps1

if |presion_izq − presion_der| < MARGEN_EMPATE (15 uds):
    girar_derecha = (ps7 >= ps0)   # decide por sensor frontal
else:
    girar_derecha = (presion_izq >= presion_der)
```

La dirección se bloquea al inicio del giro para evitar oscilaciones.

**Estado 1 — Giro Normal:** el robot gira en la dirección decidida hasta que `d̂_k < UMBRAL_SALIDA`. Si el giro supera `UMBRAL_ATASCO = 4.0 / Tₛ` pasos sin resolverse, se activa el escape.

**Estado 2 — Escape Ciego:** el robot gira más agresivamente (1.5× `VEL_GIRO`) por un tiempo limitado (`DURACION_ESCAPE_MAXIMA = 3.5 / Tₛ`). Si antes de ese límite la estimación baja de `UMBRAL_SALIDA`, regresa al avance.

**Estado 3 — Empuje Exploratorio:** tras agotar el tiempo de escape, el robot avanza brevemente (`DURACION_EMPUJE = 0.5 / Tₛ`) para salir de posibles deadlocks geométricos, con salvaguarda automática que aborta si detecta pared frontal.

---

## 8. Comparación de Señales: Cruda, Filtrada y Fusionada

### Escenario 1

La señal cruda fluctúa entre 65 y 120 unidades en un entorno de obstáculo único, con variaciones de ±5–10 uds entre muestras consecutivas que podrían generar giros innecesarios si se usara directamente como señal de control. El filtro exponencial reduce estas oscilaciones con un leve retardo de respuesta. El estimador de Kalman produce la señal más suave, con ganancia baja (K ≈ 0.39), confiando principalmente en la predicción por encoders.

### Escenario 2

En el laberinto la diferencia entre señales es más marcada. La señal cruda alcanza picos de 1048 unidades frente a obstáculos muy cercanos, mientras que el Kalman —con sintonía agresiva (K ≈ 0.92)— responde casi tan rápido como el sensor pero con mayor estabilidad. Los 60 giros registrados en 505 s demuestran una navegación fluida con muy pocas maniobras redundantes (4% del tiempo total).

| Métrica | Señal Cruda | Filtro EMA | Kalman |
|---------|------------|-----------|--------|
| Esc. 1 — Rango | 54.60 | 93.61 | 90.18 |
| Esc. 2 — Rango | 989.40 | 799.15 | 726.04 |
| Esc. 2 — Giros falsos | Elevado | Reducido | Mínimo |

---

## 9. Resultados por Escenario

### Escenario 1 — Entorno Simple

El robot completó la simulación en **10.27 s** con 322 muestras. El entorno con un único obstáculo permite apreciar claramente el ciclo avance–detección–giro–avance. La estimación de Kalman (K ≈ 0.39) siguió fielmente la tendencia de la señal filtrada, confirmando que el modelo de movimiento es preciso en línea recta. No se registraron atascos ni activación de estados de escape.

### Escenario 2 — Laberinto

El robot navegó de forma autónoma durante **505.66 s** realizando **60 giros** sin colisión. La sintonía agresiva del laberinto (Q = 2.0, R = 0.2, K ≈ 0.92) permitió al filtro responder al nivel de velocidad del sensor de distancia sin heredar su ruido. La histéresis entre `UMBRAL_CRITICO` (85) y `UMBRAL_SALIDA` (75) evitó oscilaciones de encendido/apagado del giro. Los mecanismos de anti-atasco (escape ciego + empuje) se activaron en situaciones de pasillo estrecho, permitiendo al robot salir sin intervención manual.

---

## 10. Análisis y Conclusiones

1. **Filtrado exponencial:** suaviza eficazmente el ruido de alta frecuencia de los sensores, pero introduce retardo. Con α = 0.2 el balance es adecuado para ambos entornos.

2. **Filtro de Kalman:** integra la predicción por odometría con la medición sensorial de forma óptima. La capacidad de ajustar Q y R permite adaptar el comportamiento del filtro al entorno: lento y suave en espacios abiertos, rápido y reactivo en pasillos estrechos.

3. **Campo visual ampliado:** la función `max(ps7, ps0, ps6×0.7, ps1×0.7)` anticipa obstáculos en diagonal antes de que sean frontales directos, reduciendo el riesgo de colisión en esquinas.

4. **Máquina de estados con anti-deadlock:** la jerarquía Avance → Giro → Escape → Empuje garantiza que el robot siempre pueda resolver situaciones de bloqueo geométrico sin intervención externa.

5. **Sintonía diferenciada por escenario:** los parámetros del laberinto (velocidades menores, umbrales más altos, Kalman más agresivo) demuestran que un mismo controlador puede adaptarse a entornos radicalmente distintos mediante ajuste de parámetros, sin cambiar la arquitectura.

---

## 11. Instrucciones de Ejecución

1. Clonar el repositorio en la carpeta `controllers/` del proyecto Webots:
   ```bash
   git clone <url_repositorio> controllers/lab2_controller
   ```

2. Cargar el mundo de simulación provisto (`.wbt`).

3. Asegurarse de que el controlador asignado al e-puck en el mundo sea `lab2_controller`.

4. Ejecutar la simulación en Webots. El controlador detecta automáticamente el escenario por el nombre del archivo de mundo:
   - Si el nombre contiene `"complejo"` o `"laberinto"` → usa parámetros de laberinto
   - En cualquier otro caso → usa parámetros del escenario simple

5. Al finalizar la simulación se genera automáticamente el archivo `datos_escenario1_simple.csv` o `datos_escenario2_laberinto.csv` con las columnas:

   ```
   tiempo_s, raw, filtrado_exp, kalman, avance_enc_uds, ganancia_K, estado_giro
   ```

6. Los datos pueden graficarse con cualquier herramienta compatible con CSV (Python/matplotlib, Excel, etc.).
