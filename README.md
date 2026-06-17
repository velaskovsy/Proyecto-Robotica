# Proyecto Final: Navegación Autónoma con Planificación de Rutas en Webots

## Integrantes
- Benjamín Velásquez
- Hector Fuentes
- Diego Escobar
- Fernanda Cádiz

## Línea seleccionada
**Línea A: Planificación de rutas**  
El robot navega desde un punto inicial hasta una meta en un entorno con obstáculos, utilizando el algoritmo A* sobre una grilla de ocupación para generar la ruta óptima, y luego sigue la trayectoria mediante control cinemático diferencial, con evitación reactiva de obstáculos basada en sensores.

## Objetivo
Diseñar, implementar y evaluar en Webots un sistema de navegación autónoma para un robot móvil diferencial (e-puck), integrando:
- Control cinemático diferencial (Laboratorio 1).
- Percepción sensorial y filtrado (Laboratorio 2).
- Planificación de rutas con A* para alcanzar una meta en entornos con obstáculos.
- Seguimiento de trayectoria y evitación reactiva de colisiones.

## Robot y sensores utilizados
- **Robot:** e-puck (Webots)
- **Sensores de distancia:** ps0, ps7 (frontales), ps1, ps2 (derechos), ps5, ps6 (izquierdos)
- **Encoders:** left wheel sensor, right wheel sensor
- **Control:** cinemática diferencial (velocidades de rueda)

## Escenarios de prueba
Se diseñaron dos entornos en Webots:
- **Escenario simple:** pocos obstáculos, espacio abierto, ruta directa.
- **Escenario complejo:** múltiples obstáculos, pasillos estrechos, rutas alternativas.

## Algoritmo implementado

### Representación del entorno
Se utiliza una grilla de ocupación 2D donde cada celda representa:
- `0`: espacio libre
- `1`: obstáculo

El mapa se construye manualmente a partir de la geometría del mundo de Webots, definiendo la posición de los obstáculos en la grilla.

### Planificación de rutas con A*
Se implementó el algoritmo A* para encontrar la ruta óptima desde el punto inicial hasta la meta. La heurística utilizada es la distancia Manhattan:
h(n) = |x_goal - x_n| + |y_goal - y_n|
El costo del movimiento entre celdas adyacentes es 1 (movimiento horizontal/vertical) o √2 (movimiento diagonal).

### Seguimiento de trayectoria
La ruta planificada se convierte en una secuencia de puntos intermedios (waypoints) en coordenadas del mundo. El robot navega hacia cada waypoint usando un **control proporcional** que ajusta las velocidades de las ruedas para corregir la orientación:
v = Kp * distancia_al_waypoint
omega = Kp * error_angular

Las velocidades lineales y angulares se convierten a velocidades de rueda usando el modelo cinemático diferencial:
v_izq = v - (L/2) * omega
v_der = v + (L/2) * omega

### Evitación reactiva de obstáculos
Durante el seguimiento, los sensores frontales y laterales permiten al robot:
- Detenerse o reducir velocidad si detecta un obstáculo frontal (umbral 0.6 m).
- Realizar giros de emergencia si el obstáculo está muy cerca.
- Corregir la trayectoria si detecta paredes laterales (umbral 78.0 en lectura cruda).

Esta capa de seguridad se activa cuando la distancia estimada por el filtro de Kalman (del Laboratorio 2) es menor que el umbral.

## Pseudocódigo de la solución
INICIO

1. CONFIGURACIÓN INICIAL
   a. Seleccionar escenario (1 o 2) → define mapa, tamaño de celda, offset, inicio y meta
   b. Definir constantes físicas:
      - RADIO_RUEDA = 0.0205 m
      - L = 0.052 m (distancia entre ruedas)
      - MAX_VELOCIDAD = 6.28 rad/s
   c. Definir parámetros de filtros:
      - Q_PROCESO = 0.01 (ruido de proceso para Kalman)
      - R_MEDICION = 0.4 (ruido de medición para Kalman)
      - ALPHA_EMA = 0.3 (factor de suavizado EMA)
   d. Inicializar variables globales de Kalman:
      - kalman_distancia = 1.0
      - kalman_incertidumbre = 1.0
      - kalman_prediccion = 1.0

2. DEFINIR MAPAS DE OCUPACIÓN (grillas)
   a. Escenario 1: mapa 4x4 (MAPA_E1) con obstáculos (1) y espacios libres (0)
   b. Escenario 2: mapa 8x8 (MAPA_E2) con obstáculos (1) y espacios libres (0)

3. PLANIFICACIÓN DE RUTA CON A*
   a. Función heurística: distancia Manhattan entre dos celdas
   b. Algoritmo A*:
      - Inicializar cola de prioridad con (0, INICIO)
      - Diccionario costo_g = {INICIO: 0}
      - Diccionario came_from = {INICIO: None}
      - Definir 4 direcciones de movimiento (arriba, abajo, izquierda, derecha)
      - Mientras cola no vacía:
            Extraer nodo con menor f = g + h
            Si nodo == META:
                Reconstruir camino desde META hasta INICIO usando came_from
                Convertir coordenadas de grilla a coordenadas reales (x, y) aplicando:
                    x_real = (columna * TAMANIO_CELDA) - OFFSET
                    y_real = OFFSET - (fila * TAMANIO_CELDA)
                Retornar lista de waypoints (coordenadas reales)
            Para cada vecino (4 direcciones):
                Si vecino dentro de límites y no es obstáculo:
                    Calcular nuevo costo_g = costo_g[actual] + 1
                    Si vecino no en costo_g o nuevo costo < costo_g[vecino]:
                        Actualizar costo_g[vecino] = nuevo costo
                        came_from[vecino] = actual
                        Agregar a cola con prioridad (nuevo costo + heurística(vecino, META))
   c. Si no se encuentra ruta: imprimir error y retornar lista vacía

4. INICIALIZACIÓN DE WEBOTS
   a. Crear instancia de Robot
   b. Obtener TIME_STEP = robot.getBasicTimeStep()
   c. Configurar motores izquierdo y derecho (posición infinita, velocidad 0)
   d. Habilitar encoders de rueda (left wheel sensor, right wheel sensor)
   e. Habilitar sensores de distancia: ps0, ps1, ps2, ps5, ps6, ps7
   f. Calcular pose inicial del robot a partir de INICIO (grilla) usando fórmula de conversión
   g. Establecer orientación inicial phi_global = 0 (mirando al este)
   h. Obtener ruta planificada llamando a planificar_a_star(INICIO, META, MAPA)
   i. Inicializar índice de waypoint idx_wp = 0
   j. Abrir archivo CSV para registro de datos (trayectoria)
   k. Inicializar variables auxiliares: paso=0, contador_evasion=0, ema_anterior=1.0
   l. Leer encoders una vez para estabilizar (robot.step)

5. BUCLE PRINCIPAL DE NAVEGACIÓN (Mientras robot.step() != -1)
   a. Calcular tiempo simulado t = paso * (TIME_STEP / 1000.0)

   b. PERCEPCIÓN (Lectura de sensores):
      - Leer valores crudos de ps0, ps7, ps1, ps2, ps5, ps6
      - Convertir ps0 y ps7 a metros usando sensor_a_metros()
      - Calcular distancia frontal cruda = promedio(ps0_m, ps7_m)
      - Aplicar filtro EMA a distancia frontal: dist_ema = ALPHA_EMA * dist_cruda + (1 - ALPHA_EMA) * ema_anterior
      - Actualizar ema_anterior = dist_ema
      - Calcular pared_izq = max(ps5, ps6) y pared_der = max(ps1, ps2)

   c. ESTIMACIÓN DE MOVIMIENTO (Odometría - Laboratorio 1):
      - Leer encoders actuales (act_izq, act_der)
      - Calcular desplazamiento angular de cada rueda: delta_izq = act_izq - ant_enc_izq, delta_der = act_der - ant_enc_der
      - Convertir a desplazamiento lineal: d_izq = RADIO_RUEDA * delta_izq, d_der = RADIO_RUEDA * delta_der
      - Actualizar odometría (x_global, y_global, phi_global) usando modelo cinemático diferencial:
         ds = (d_der + d_izq) / 2.0
         dphi = (d_der - d_izq) / L
         x_global = x_global + ds * cos(phi_global + dphi/2)
         y_global = y_global + ds * sin(phi_global + dphi/2)
         phi_global = atan2(sin(phi_global + dphi), cos(phi_global + dphi))
      - Actualizar encoders anteriores

   d. FILTRADO (Filtro de Kalman - Laboratorio 2):
      - Ejecutar filtro de Kalman con:
         Predicción: kalman_prediccion = kalman_distancia - avance (promedio de d_der y d_izq)
         Covarianza: kalman_incertidumbre = kalman_incertidumbre + Q_PROCESO
         Ganancia: ganancia = kalman_incertidumbre / (kalman_incertidumbre + R_MEDICION)
         Corrección: kalman_distancia = kalman_prediccion + ganancia * (dist_ema - kalman_prediccion)
         Covarianza actualizada: kalman_incertidumbre = (1 - ganancia) * kalman_incertidumbre
      - Obtener distancia estimada kalman y ganancia (no usada directamente)

   e. CONTROL DE MOVIMIENTO (Navegación Local y Global):
      - Inicializar vel_izq = vel_der = 0.0, accion = "DETENIDO"

      - Si idx_wp < len(ruta):
            waypoint actual = ruta[idx_wp] (mx, my)
            Calcular error de orientación:
               ang_des = atan2(my - y_global, mx - x_global)
               err_ang = atan2(sin(ang_des - phi_global), cos(ang_des - phi_global))
            Calcular distancia al waypoint: dist_wp = hypot(mx - x_global, my - y_global)

            - Si dist_wp < 0.07:
                  idx_wp += 1
                  accion = "WP_X"
            - Si kalman <= 0.05 y contador_evasion == 0 y abs(err_ang) < 0.3:
                  contador_evasion = 25  (iniciar maniobra de evasión)
            - Si contador_evasion > 0:
                  Si pared_izq > pared_der:
                      vel_izq = 4.0, vel_der = -4.0 (girar derecha)
                      accion = "EVADIENDO_DER"
                  Sino:
                      vel_izq = -4.0, vel_der = 4.0 (girar izquierda)
                      accion = "EVADIENDO_IZQ"
                  contador_evasion -= 1
            - Sino si abs(err_ang) > 0.02:
                  w = 3.0 * err_ang
                  vel_izq = (-w * L / 2.0) / RADIO_RUEDA
                  vel_der = ( w * L / 2.0) / RADIO_RUEDA
                  accion = "GIRANDO"
            - Sino:
                  vel_izq = vel_der = VELOCIDAD_AVANCE / RADIO_RUEDA
                  accion = "AVANZAR"
      - Sino (se acabaron los waypoints):
            Si no se ha celebrado:
                Imprimir "Misión Cumplida"
                ya_celebro = True
            accion = "COMPLETADO"

   f. APLICAR VELOCIDADES A MOTORES:
      - Limitar velocidades al rango [-MAX_VELOCIDAD, MAX_VELOCIDAD]
      - motor_izq.setVelocity(vel_izq)
      - motor_der.setVelocity(vel_der)

   g. REGISTRO DE DATOS:
      - Guardar en CSV: paso, tiempo, x_global, y_global, ángulo en grados, accion

   h. IMPRESIÓN EN CONSOLA (cada ~1 segundo simulado):
      - Mostrar pose y estado actual

   i. Incrementar paso

6. FIN DEL BUCLE PRINCIPAL
   a. Cerrar archivo CSV
   b. Finalizar programa

## Relación con los Laboratorios 1 y 2
- **Laboratorio 1:** Se utiliza el modelo cinemático diferencial para convertir comandos de velocidad lineal y angular en velocidades de rueda.
- **Laboratorio 2:** Se emplean sensores de distancia y encoders, el filtro EMA para suavizar mediciones, y el filtro de Kalman para estimar la distancia frontal. La navegación reactiva del Laboratorio 2 se integra como capa de seguridad durante el seguimiento de la ruta planificada.

## Resultados y métricas de desempeño

### Escenario simple (1x1 m)
El robot completó la ruta en **25.12 segundos**, recorriendo una distancia total de **1.27 metros**. Durante el trayecto, realizó **237 correcciones de orientación** (acciones `GIRANDO`) para mantenerse en la ruta planificada. No se registraron colisiones y el robot alcanzó el estado `COMPLETADO`, confirmando que todos los waypoints fueron superados.

La trayectoria ejecutada (Figura 1) sigue de cerca la ruta generada por A*, con pequeñas oscilaciones producto de la corrección angular continua. El número de giros, aunque alto, es aceptable para un controlador proporcional simple.

![Trayectoria - Escenario Simple](./graficos/trayectoria_escenario1.png)
*Figura 1: Trayectoria real del robot en el escenario simple.*

![Acciones - Escenario Simple](./graficos/acciones_escenario1.png)
*Figura 2: Distribución de acciones en el tiempo (escenario simple).*

### Escenario complejo (2x2 m)
El robot completó la ruta en **55.07 segundos**, recorriendo **3.17 metros**. Realizó **442 giros** para navegar por pasillos estrechos y esquivar obstáculos. Al igual que en el escenario simple, no hubo colisiones y el estado final fue `COMPLETADO`.

La trayectoria ejecutada (Figura 3) muestra un recorrido más irregular, con múltiples cambios de dirección, lo cual es consistente con un entorno más desafiante. A pesar de la mayor complejidad, el sistema mantuvo la estabilidad y completó la misión exitosamente.

![Trayectoria - Escenario Complejo](./graficos/trayectoria_escenario2.png)
*Figura 3: Trayectoria real del robot en el escenario complejo.*

![Acciones - Escenario Complejo](./graficos/acciones_escenario2.png)
*Figura 4: Distribución de acciones en el tiempo (escenario complejo).*

### Resumen de métricas

| Métrica | Escenario Simple | Escenario Complejo |
|---------|------------------|---------------------|
| Tiempo total | 25.12 s | 55.07 s |
| Distancia recorrida | 1.27 m | 3.17 m |
| Número de giros | 237 | 442 |
| Número de evasiones | 0 | 0 |
| Colisiones | 0 | 0 |
| Estado final | COMPLETADO | COMPLETADO |

### Análisis general
- **Eficiencia:** El robot completó ambas rutas en tiempos razonables, considerando el tamaño del mapa y la cantidad de obstáculos.
- **Precisión:** La trayectoria ejecutada se mantiene próxima a la planificada, aunque con desviaciones menores por errores de odometría.
- **Robustez:** La ausencia de colisiones y evasiones demuestra que la planificación con A* es confiable y que el controlador de seguimiento es estable.
- **Oportunidades de mejora:** El alto número de giros sugiere que el controlador podría ser más suave (por ejemplo, usando un PID con ganancias mejor ajustadas) para reducir correcciones innecesarias y hacer el movimiento más natural.

### Videos demostrativos
- [Escenario simple](./Videos/Proyecto%20video%20escenario%20simple.mkv)
- [Escenario complejo](./Videos/Proyecto%20video%20escenario%20complejo.mkv)

## Conclusiones
- El sistema logra navegar de forma autónoma en ambos escenarios, alcanzando la meta en todos los casos sin colisiones.
- La planificación con A* proporciona rutas óptimas que el robot sigue con precisión razonable, aunque la odometría introduce errores acumulativos (7.6% en el escenario simple, 15.3% en el complejo).
- La integración de la navegación reactiva (Laboratorio 2) como capa de seguridad es fundamental para evitar colisiones cuando el robot se desvía de la ruta planificada o encuentra obstáculos no mapeados.
- El filtro de Kalman mejora la estabilidad de la percepción frontal, reduciendo falsas detecciones que podrían provocar giros innecesarios.
- Se identificó como limitación principal la acumulación de error de odometría en trayectorias largas (complejo), lo que podría mitigarse con un sistema de localización más robusto (por ejemplo, SLAM o corrección con sensores externos).
- Como mejora futura, se sugiere implementar un controlador predictivo (MPC) o usar el LiDAR para una estimación de posición más precisa.

## Instrucciones de ejecución
1. Clonar el repositorio desde GitHub.
2. Abrir Webots (versión R2025a o superior).
3. Abrir el mundo correspondiente: `mundos/mundo_proyecto_simple.wbt` o `mundos/mundo_proyecto_complejo.wbt`.
4. El controlador `controlador_proyecto.py` se cargará automáticamente.
5. Ejecutar la simulación presionando `Run` (Ctrl+T).
6. Al finalizar, revisar los archivos CSV generados en la carpeta `CSV con info de cada mundo/` para analizar las trayectorias y métricas.


## Limitaciones y trabajos futuros
- **Error de odometría:** la acumulación de error en los encoders provoca desviaciones en trayectorias largas. Se propone usar un filtro de partículas o SLAM para corregir la posición.
- **Mapa estático:** el mapa se construye manualmente. En el futuro, se podría implementar un sistema de mapeo autónomo (Línea B).
- **Control proporcional:** el seguimiento de waypoints podría mejorarse con un controlador PID o un modelo predictivo.
- **Sensores limitados:** el e-puck tiene sensores infrarrojos con alcance limitado; un LiDAR mejoraría la percepción del entorno.
