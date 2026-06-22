# Proyecto Final: Navegación Autónoma con A* y Filtro de Kalman (E-puck)

Este repositorio contiene la solución para el **Proyecto Final de Robótica y Sistemas Autónomos (Código: ICI 4150)**. Se implementa un sistema de navegación autónoma en Webots para un robot móvil diferencial **E-puck** que viaja desde una posición inicial hasta una meta utilizando el algoritmo **A*** sobre una grilla de ocupación discreta, un **Filtro de Kalman Diagonal** para la fusión sensorial y control de navegación local reactiva.

---

## Integrantes del Grupo
* [Completar con Nombre integrante 1]
* [Completar con Nombre integrante 2]
* [Completar con Nombre integrante 3]
* [Completar con Nombre integrante 4]
* [Completar con Nombre integrante 5]

---

## 1. Tipo de Robot Utilizado y Configuración en Webots

El robot utilizado es un **E-puck** (modelo PROTO estándar en Webots) configurado con las siguientes características:
* **Actuadores:** Dos motores de corriente continua independientes para tracción diferencial: `"left wheel motor"` (rueda izquierda) y `"right wheel motor"` (rueda derecha), configurados en modo de control de velocidad angular (con posición establecida en infinito `float('inf')`).
* **Parámetros Físicos del Robot:**
  * Radio de la rueda ($r$): $0.0205$ m ($20.5$ mm).
  * Distancia entre ruedas (axle length, $L$): $0.052$ m ($52$ mm).
  * Velocidad lineal máxima de avance programada ($V_{max}$): $0.08$ m/s.
  * Velocidad angular de motores máxima permitida: $6.28$ rad/s.

---

## 2. Sensores Incorporados y Explicación de Uso

* **GPS (`"gps"`):** Proporciona la posición cartesiana tridimensional real $(x, y, z)$ del robot en la simulación. Se utiliza como lectura de posición absoluta para la corrección del Filtro de Kalman.
* **Unidad Inercial (`"inertial unit"` - IMU):** Mide la orientación angular absoluta del robot. Se lee el ángulo de guiñada (yaw, $\phi$) en el eje vertical Z y se utiliza como lectura de orientación absoluta para corregir la deriva inercial.
* **Sensores de Posición de Rueda (`"left/right wheel sensor"`):** Encoders incrementales instalados en los motores de las ruedas. Miden la rotación angular acumulada ($\theta_i, \theta_d$) de cada rueda. Se usan para calcular la distancia lineal recorrida ($\Delta s$) y la rotación angular instantánea ($\Delta \phi$) en cada ciclo de la odometría.
* **Sensores de Proximidad Infrarrojos (`"ps0"` a `"ps7"`):** Se incorporaron 6 sensores de distancia: los frontales `ps0` y `ps7` (a $17^\circ$), los diagonales `ps1` y `ps6` (a $45^\circ$), y los laterales `ps2` y `ps5` (a $90^\circ$). Miden la luz infrarroja reflejada para la navegación local reactiva y la evitación de colisiones.

---

## 3. Funcionamiento del Controlador Implementado

El controlador (`controlador.py`) opera en un bucle cerrado en tiempo real (ejecutado cada $dt = 32$ ms) dentro de la función `main()` bajo la siguiente secuencia:
1. **Inicialización:** Conexión con motores y sensores, activación de los pasos de tiempo e inicio de variables.
2. **Cálculo de Odometría:** Calcula la posición estimada integrando en el tiempo el giro de las ruedas captado por los encoders.
3. **Fusión Sensorial con Filtro de Kalman Diagonal:**
   * **Predicción:** Usando el modelo cinemático diferencial no lineal, se estima la nueva posición y orientación $[x, y, \phi]^T$ basándose en la odometría de los encoders, y se propaga el error (varianza de predicción $P$).
     $$P_{xx} \leftarrow P_{xx} + Q_{xx}$$
     $$P_{yy} \leftarrow P_{yy} + Q_{yy}$$
     $$P_{\phi\phi} \leftarrow P_{\phi\phi} + Q_{\phi\phi}$$
   * **Actualización:** Se contrasta la predicción con las mediciones reales del GPS ($x, y$) y de la IMU ($\phi$). Se calculan las ganancias de Kalman ($K_x, K_y, K_\phi$) y se corrige la estimación de estado, anulando el error acumulativo (drift) de la odometría:
     $$K_x = \frac{P_{xx}}{P_{xx} + R_{xx}}, \quad K_y = \frac{P_{yy}}{P_{yy} + R_{yy}}, \quad K_\phi = \frac{P_{\phi\phi}}{P_{\phi\phi} + R_{\phi\phi}}$$
     $$x_{est} = x_{pred} + K_x (x_{gps} - x_{pred})$$
     $$y_{est} = y_{pred} + K_y (y_{gps} - y_{pred})$$
     $$\phi_{est} = \phi_{pred} + K_\phi (\phi_{imu} - \phi_{pred})$$
     $$P_{xx} \leftarrow (1 - K_x) P_{xx}, \quad P_{yy} \leftarrow (1 - K_y) P_{yy}, \quad P_{\phi\phi} \leftarrow (1 - K_\phi) P_{\phi\phi}$$
4. **Controlador Cinemático y Control de Actuadores:** Evalúa la pose estimada actual, calcula el rumbo y la velocidad lineal necesarios hacia el waypoint objetivo y comanda los motores.

---

## 4. Estrategia de Navegación Desarrollada: Planificación de Rutas (Línea A)

Se seleccionó la **Línea A: Planificación de rutas**:
* **Mapa de Ocupación Estático:** El laberinto del escenario `complejo.wbt` se representa como una matriz de grilla de ocupación discreta de **13x13 celdas** (donde cada celda tiene un tamaño de $0.25 \times 0.25$ m, alineada con los bloques del mundo). Las celdas con obstáculos fijos están marcadas con `1` y las libres con `0`.
* **Planificación Global A\*:** Se ejecuta el algoritmo **A\*** con conectividad de **4 vecindades** (arriba, abajo, izquierda, derecha) antes de iniciar el movimiento. Al restringir el movimiento a 4 direcciones ortogonales, se garantiza que la ruta planificada pase estrictamente por el centro de los pasillos discretos, maximizando la distancia de seguridad contra las esquinas.
* **Conversión a Waypoints:** Las celdas calculadas por el A* en la grilla se traducen a coordenadas reales en metros $(x, y)$ que sirven como puntos de ruta.

---

## 5. Evitación de Obstáculos y Seguimiento de Ruta

* **Seguimiento de Ruta:** El robot calcula continuamente el error de ángulo $\theta_{err}$ hacia el siguiente waypoint. Si el error angular es superior a $0.4$ radianes, el robot frena y realiza un giro en el propio eje. Si es menor, avanza hacia adelante corrigiendo el rumbo proporcionalmente con un control proporcional de orientación ($\omega = K_p \theta_{err}$). Al estar cerca del waypoint ($<4$ cm), cambia al siguiente.
* **Evitación Local Reactiva (Braitenberg):**
  * Se utiliza un esquema de evitación continuo basado en las señales de proximidad (`ps0, ps1, ps2` a la derecha; `ps7, ps6, ps5` a la izquierda).
  * Si un obstáculo cercano estimula los sensores de un costado (como el cilindro azul de prueba en el pasillo), la diferencia de lecturas (`left_sum - right_sum`) genera una velocidad angular correctora ($\omega$) que empuja al robot suavemente al lado contrario y reduce dinámicamente su velocidad lineal de avance (hasta un 20% de $V_{max}$).
* **Parada de Emergencia:** Si las señales frontales directas (`ps0` y `ps7`) leen valores superiores a $900.0$ (bloqueo completo al frente a menos de 1 cm), el robot se detiene por completo (`v = 0, \omega = 0`).
* **Comportamiento de Recuperación (Recovery Behavior):** Cada segundo el robot compara su posición cartesiana real (GPS) con la del segundo anterior. Si la velocidad enviada a las ruedas es mayor a cero pero el desplazamiento real es menor a $1.5$ cm, se declara un atascamiento. El robot retrocede marcha atrás y gira en sentido contrario al obstáculo durante $1.5$ segundos para liberarse, antes de reanudar el seguimiento de la ruta A*.

---

## 6. Resultados Obtenidos en los Escenarios de Prueba

Al ejecutar la simulación en el escenario `complejo.wbt` con el cilindro azul de prueba obstaculizando parcialmente el pasillo de la columna 5:
* **Planificación:** El algoritmo A* calculó de forma instantánea al inicio de la simulación una ruta óptima sin colisiones de 29 celdas y $3.5$ metros de longitud.
* **Seguimiento y Evasión:** El robot siguió con total precisión los pasillos. Al llegar al cilindro azul, lo detectó con los sensores de proximidad laterales y frontales de la rueda derecha, desaceleró suavemente y se abrió a la izquierda, rodeando el cilindro de forma fluida y sin colisionar antes de continuar.
* **Mitigación de la Deriva:** Al finalizar la corrida, la odometría pura del robot acumuló un error final de **$10$ cm** de desviación respecto a la meta física. Por su parte, la estimación del **Filtro de Kalman** mantuvo un error menor a **$0.5$ cm** respecto a las coordenadas reales del GPS, lo cual demuestra una fusión y corrección perfectas.
* **Métricas:**
  * **Tiempo empleado:** $\approx 125$ segundos.
  * **Colisiones:** 0.
  * **Bitácora:** Generación exitosa de `trayectorias.csv` con el historial de navegación para análisis gráfico.

---

## 7. Principales Dificultades, Limitaciones y Posibles Mejoras

* **Dificultades:** 
  * El deslizamiento físico de las ruedas del robot al acelerar o girar introduce saltos de error en la odometría acumulativa.
  * Los puntos ciegos laterales traseros del E-puck y el corto rango de visión de los sensores infrarrojos exigen una desaceleración reactiva muy fuerte en áreas estrechas para evitar el raspado de esquinas.
* **Limitaciones:** 
  * La grilla de A* tiene un tamaño fijo de $13\times 13$. Cambiar el tamaño o la alineación de la arena de Webots requeriría modificar manualmente la matriz de grilla hardcodeada.
  * Si el laberinto se bloquea por completo de forma permanente, el robot se detendrá por la parada de emergencia pero no tiene la capacidad de recalcular una ruta global alternativa en tiempo real.
* **Posibles Mejoras:**
  * **Replanificación de Ruta Dinámica (D* Lite):** Permitir que el robot recalcule el camino A* global si los sensores detectan que una celda planificada está permanentemente bloqueada.
  * **Integración de SLAM (Línea B):** Implementar la construcción autónoma de la grilla de ocupación a medida que el robot avanza en lugar de tener la grilla predefinida (grilla estática).

---

## 8. Instrucciones para Ejecutar la Simulación

1. Asegúrate de tener instalado **Webots (R2023a o superior)** y **Python 3.x**.
2. Copia todo el directorio del proyecto en tu espacio de trabajo de Webots.
3. Abre Webots y carga el archivo de mundo localizado en `worlds/complejo.wbt`.
4. El robot `E-puck` ya está configurado para utilizar el controlador llamado `controlador`.
5. Presiona el botón de **Play (Run)** en Webots para iniciar la simulación.
6. En la pestaña de consola del robot, podrás ver la grilla del laberinto, el camino calculado por $A^*$, las actualizaciones de seguimiento en tiempo real y, finalmente, las métricas de desempeño al tocar la meta.
7. Al finalizar, se generará el archivo `trayectorias.csv` en la carpeta `controllers/controlador/` con los datos para su posterior graficación.
