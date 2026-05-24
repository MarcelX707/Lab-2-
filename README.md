# Laboratorio 2: Navegación Reactiva con Filtrado y Fusión Sensorial

Asignatura: Robótica y Sistemas Autónomos 2026-01   
Código: ICI 4150   
Institución: Pontificia Universidad Católica de Valparaíso (PUCV)
## Objetivo
El objetivo de este laboratorio es implementar un sistema de navegación reactiva en Webots para un robot móvil diferencial. Se busca utilizar sensores de distancia y encoders de rueda para aplicar técnicas de filtrado y un Filtro de Kalman que permita estimar la distancia frontal a obstáculos y mejorar la toma de decisiones autónoma.
## 2. Descripción del Robot y Sensores
Se utilizó un robot móvil diferencial (e-puck). La configuración de hardware interna del controlador considera los siguientes dispositivos:
* Sensores de Proximidad Frontales: ps7 y ps0 (utilizados para la fusión sensorial).
* Sensores Diagonales: ps6 y ps1 (para apoyo en detección de esquinas).
* Sensores Laterales: ps5 (izquierdo) y ps2 (derecho), empleados para decidir la dirección de giro
* Encoders de Rueda: Utilizados para estimar el desplazamiento lineal del robot mediante la rotación de los motores.

## 3. Frecuencia de Muestreo
Los registros y cálculos se realizan con una frecuencia de muestreo fija asociada al paso de simulación de Webots.
* Tiempo de muestreo ($T_s$): $0.032$ s.
* Frecuencia de muestreo ($f_s$): $31.25$ Hz.

## 4. Implementación del Sistema
### 4.1. Estimación de Avance (Odometría)
Se utiliza la relación cinemática para convertir el desplazamiento angular de los encoders ($\theta$) en desplazamiento lineal ($s$), considerando el radio de la rueda ($r = 0.0205$ m):
* 
### 4.2. Filtrado Simple
Se implementó un filtro exponencial (paso bajo) sobre las señales crudas de los sensores frontales para suavizar el ruido de medición antes de ingresarlas al estimador principal:


### 4.3. Filtro de Kalman (Fusión Sensorial)
El filtro de Kalman se encarga de fusionar la predicción del movimiento con la corrección del entorno:
1. Etapa de Predicción: Se estima la nueva proximidad basándose en el avance detectado por los encoders. Si el robot avanza, la proximidad al objeto aumenta.
2. Etapa de Corrección: Se ajusta la predicción utilizando la lectura del sensor frontal. La Ganancia de Kalman ($K_k$) pondera ambas fuentes según su incertidumbre.

## 5. Lógica de Navegación Reactiva
La toma de decisiones utiliza la distancia frontal estimada para garantizar estabilidad:
* Avance: El robot mantiene VEL_CRUCERO mientras la proximidad estimada sea segura
* Evasión: Al superar el UMBRAL_CRITICO, el robot activa un giro.
* Dirección de Giro: Si los sensores izquierdos detectan mayor proximidad que los derechos, el robot gira a la derecha para alejarse del obstáculo, y viceversa.

## 6. Resultados y Análisis
El análisis de las señales registradas permite concluir:
* Señal Cruda: Presenta fluctuaciones ruidosas que podrían generar giros innecesarios.
* Filtro de Kalman: Proporciona una estimación más estable y confiable, permitiendo una navegación suave y reduciendo colisiones en entornos estrechos.

## 7. Instrucciones de Ejecución
1. Clonar el repositorio en la carpeta controllers/ de su proyecto Webots.
2. Cargar el mundo de simulación (.wbt) provisto.
3. Ejecutar la simulación para visualizar el comportamiento reactivo y la generación del archivo datos_lab2.csv.



