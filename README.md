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

## Relación con los Laboratorios 1 y 2
- **Laboratorio 1:** Se utiliza el modelo cinemático diferencial para convertir comandos de velocidad lineal y angular en velocidades de rueda.
- **Laboratorio 2:** Se emplean sensores de distancia y encoders, el filtro EMA para suavizar mediciones, y el filtro de Kalman para estimar la distancia frontal. La navegación reactiva del Laboratorio 2 se integra como capa de seguridad durante el seguimiento de la ruta planificada.

## Resultados

### Escenario simple

#### Métricas de desempeño

| Métrica | Valor |
|---------|-------|
| Tiempo total hasta la meta | 25.12 s |
| Posición inicial | (-0.375, 0.375) m |
| Posición final (meta) | (0.359, -0.3104) m |
| Distancia directa (meta) | 1.004 m |
| Longitud trayectoria ejecutada | ≈1.08 m |
| Diferencia (error de trayectoria) | ≈7.6% (0.08 m extra) |
| Número de colisiones | 0 |
| Número de giros principales | 4 |
| Estado final | COMPLETADO (meta alcanzada) |

#### Análisis
El robot parte de la posición inicial y avanza en línea recta, luego realiza una serie de giros suaves para ajustar su orientación y dirigirse hacia la meta. La trayectoria ejecutada sigue de cerca la ruta planificada, con pequeñas desviaciones debido a la odometría. Se registran 4 giros principales, todos necesarios para sortear los obstáculos del escenario. El robot alcanza la meta sin colisiones en 25.12 segundos.

### Escenario complejo

#### Métricas de desempeño

| Métrica | Valor |
|---------|-------|
| Tiempo total hasta la meta | 55.07 s |
| Posición inicial | (-0.875, 0.875) m |
| Posición final (meta) | (0.8752, -0.8101) m |
| Distancia directa (meta) | 2.429 m |
| Longitud trayectoria ejecutada | ≈2.80 m |
| Diferencia (error de trayectoria) | ≈15.3% (0.37 m extra) |
| Número de colisiones | 0 |
| Número de giros principales | 8 |
| Estado final | COMPLETADO (meta alcanzada) |

#### Análisis
El escenario complejo presenta más obstáculos y pasillos estrechos, lo que obliga al robot a realizar múltiples correcciones de trayectoria. Se registran 8 giros principales, muchos de ellos para navegar por corredores y evitar obstáculos laterales. La odometría introduce un error acumulado mayor (15.3% de desviación), pero la capa de navegación reactiva (sensores de distancia) permite al robot mantenerse dentro del camino y evitar colisiones. El robot alcanza la meta en 55.07 segundos sin colisiones.

## Métricas de desempeño (resumen)

| Métrica | Escenario simple | Escenario complejo |
|---------|------------------|---------------------|
| Tiempo total | 25.12 s | 55.07 s |
| Longitud directa (meta) | 1.004 m | 2.429 m |
| Longitud trayectoria ejecutada | 1.08 m | 2.80 m |
| Error de trayectoria | 7.6% | 15.3% |
| Colisiones | 0 | 0 |
| Giros principales | 4 | 8 |
| Estado | COMPLETADO | COMPLETADO |

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

## Video demostrativo
- [Escenario simple](Vídeos/Proyecto%20video%20escenario%20simple.mkv)
- [Escenario complejo](Vídeos/Proyecto%20video%20escenario%20complejo.mkv)

## Limitaciones y trabajos futuros
- **Error de odometría:** la acumulación de error en los encoders provoca desviaciones en trayectorias largas. Se propone usar un filtro de partículas o SLAM para corregir la posición.
- **Mapa estático:** el mapa se construye manualmente. En el futuro, se podría implementar un sistema de mapeo autónomo (Línea B).
- **Control proporcional:** el seguimiento de waypoints podría mejorarse con un controlador PID o un modelo predictivo.
- **Sensores limitados:** el e-puck tiene sensores infrarrojos con alcance limitado; un LiDAR mejoraría la percepción del entorno.
