"""
Controlador Proyecto Final: SLAM / Planificación de Rutas (Línea A)
Robótica y Sistemas Autónomos
"""
import math
import csv
import heapq
import os
from controller import Robot

# CONFIGURACIÓN GENERAL Y SELECTOR DE ESCENARIO
# Selector rápido: 1 = Escenario Simple (1x1m), 2 = Escenario Complejo (2x2m)
ESCENARIO = 2

# Parámetros físicos del robot 
RADIO_RUEDA   = 0.0205
L             = 0.052  # Distancia entre ruedas
MAX_VELOCIDAD = 6.28

# Parámetros matemáticos del Filtro de Kalman 
Q_PROCESO     = 0.01
R_MEDICION    = 0.4
ALPHA_EMA     = 0.3

# Variables de estado global para Kalman
kalman_distancia     = 1.0
kalman_incertidumbre = 1.0
kalman_prediccion    = 1.0

# MAPAS Y ALINEACIÓN DE COORDENADAS
# Grilla 4x4, escenario simple
MAPA_E1 = [
    [0, 0, 1, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0]
]

# Grilla 8x8, escenario complejo
MAPA_E2 = [
    [0, 0, 0, 0, 1, 1, 0, 0],
    [1, 1, 1, 0, 1, 0, 0, 1],
    [0, 0, 0, 0, 0, 0, 1, 1],
    [0, 1, 1, 1, 1, 0, 0, 0],
    [0, 0, 0, 1, 0, 0, 1, 0],
    [1, 1, 0, 1, 0, 1, 1, 0],
    [0, 0, 0, 0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0, 0, 0, 0],
]

# Configuración dinámica de variables según el escenario seleccionado
if ESCENARIO == 1:
    MAPA          = MAPA_E1
    TAMANIO_CELDA = 0.25
    OFFSET        = 0.375  # Desfase matemático para alinear con el centro de Webots
    INICIO        = (0, 0)
    META          = (3, 3)
else:
    MAPA          = MAPA_E2
    TAMANIO_CELDA = 0.25
    OFFSET        = 0.875  
    INICIO        = (0, 0)
    META          = (7, 7)

VELOCIDAD_AVANCE = 0.08

# FUNCIONES: SENSORES, FILTROS Y ODOMETRÍA, DADOS POR LABORATORIOS 1 Y 2
def sensor_a_metros(v):
    # Convierte el valor crudo del sensor IR a metros.
    return (0.05 * 1024.0) / max(v, 1.0)

def encoder_a_lineal(d):
    # Convierte el delta de ángulo del encoder a distancia lineal recorrida.
    return RADIO_RUEDA * d

def filtro_ema(nuevo, anterior):
     # Suavizado de lectura de sensores mediante Media Móvil Exponencial.
    return ALPHA_EMA * nuevo + (1 - ALPHA_EMA) * anterior

def ejecutar_filtro_kalman(avance, medicion):
    # Aplica el Filtro de Kalman usando la odometría como predicción.
    global kalman_distancia, kalman_incertidumbre, kalman_prediccion
    kalman_prediccion    = kalman_distancia - avance
    kalman_incertidumbre = kalman_incertidumbre + Q_PROCESO
    ganancia             = kalman_incertidumbre / (kalman_incertidumbre + R_MEDICION)
    kalman_distancia     = kalman_prediccion + ganancia * (medicion - kalman_prediccion)
    kalman_incertidumbre = (1 - ganancia) * kalman_incertidumbre
    return kalman_distancia, ganancia

def calcular_odometria(x, y, phi, d_der, d_izq):
    # Estimación de pose basada en cinemática diferencial del Lab 1.
    ds   = (d_der + d_izq) / 2.0
    dphi = (d_der - d_izq) / L
    x    = x + ds * math.cos(phi + dphi / 2.0)
    y    = y + ds * math.sin(phi + dphi / 2.0)
    phi  = math.atan2(math.sin(phi + dphi), math.cos(phi + dphi))
    return x, y, phi

# Funciones de utilidad para exportación de datos 
def crear_csv(nombre):
    ruta = os.path.join(os.path.dirname(__file__), nombre)
    f = open(ruta, "w", newline="")
    w = csv.writer(f)
    w.writerow(["step","time_s","x_robot","y_robot","angulo_grados","action"])
    return f, w

def guardar_csv(w, paso, t, x, y, ang, accion):
    w.writerow([paso, round(t,3), round(x,4), round(y,4), round(ang,2), accion])


# ALGORITMO A* DE NAVEGACIÓN GLOBAL
def heuristica(a, b):
    # Calcula la distancia Manhattan entre dos nodos de la grilla.
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def planificar_a_star(inicio, meta, mapa):
    # Genera la ruta óptima sorteando los obstáculos de la matriz.
    filas, cols = len(mapa), len(mapa[0])
    heap = [(0, inicio)]
    costo_g   = {inicio: 0}
    came_from = {inicio: None}
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    
    while heap:
        _, actual = heapq.heappop(heap)
        
        # Ruta encontrada
        if actual == meta:
            camino = []
            n = meta
            while n: camino.append(n); n = came_from[n]
            camino.reverse()
            
            ruta = []
            # Transformación de las coordenadas de la grilla al espacio real en Webots
            for (fila, col) in camino:
                x = (col * TAMANIO_CELDA) - OFFSET
                y = OFFSET - (fila * TAMANIO_CELDA)
                ruta.append((x, y))
                
            print(f"Ruta A*: {len(ruta)} waypoints -> {camino}")
            return ruta
            
        # Exploración de vecinos
        for df,dc in dirs:
            v = (actual[0]+df, actual[1]+dc)
            if not (0<=v[0]<filas and 0<=v[1]<cols): continue
            if mapa[v[0]][v[1]] == 1: continue # Casilla ocupada
            
            ng = costo_g[actual] + 1
            if v not in costo_g or ng < costo_g[v]:
                costo_g[v]   = ng
                came_from[v] = actual
                heapq.heappush(heap, (ng + heuristica(v, meta), v))
                
    print("Error: No se encontró ruta al objetivo.")
    return []

# INICIALIZACIÓN DE WEBOTS Y SENSORES
robot     = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

motor_izq = robot.getDevice("left wheel motor")
motor_der = robot.getDevice("right wheel motor")
motor_izq.setPosition(float("inf"))
motor_der.setPosition(float("inf"))
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

enc_izq = robot.getDevice("left wheel sensor")
enc_der = robot.getDevice("right wheel sensor")
enc_izq.enable(TIME_STEP)
enc_der.enable(TIME_STEP)

# Activamos los 6 sensores IR necesarios, en este caso serán los frontales y laterales
sensores = {}
for nombre in ["ps0","ps1","ps2","ps5","ps6","ps7"]:
    s = robot.getDevice(nombre)
    s.enable(TIME_STEP)
    sensores[nombre] = s

# Pose inicial alineada a la primera celda del mapa, mirando al este
x_global   = (INICIO[1] * TAMANIO_CELDA) - OFFSET
y_global   = OFFSET - (INICIO[0] * TAMANIO_CELDA)
phi_global = 0  

# Planeando la ruta
ruta              = planificar_a_star(INICIO, META, MAPA)
idx_wp            = 0
archivo, escritor = crear_csv(f"datos_trayectoria_escenario{ESCENARIO}.csv")

# Variables de control temporal
paso         = 0
contador_evasion = 0
ya_celebro   = False
ema_anterior = 1.0

# Lectura inicial para estabilizar encoders
robot.step(TIME_STEP)
ant_enc_izq = enc_izq.getValue()
ant_enc_der = enc_der.getValue()

print(f"Escenario {ESCENARIO} iniciado. Robot listo.")

# ------------------------- MAIN --------------------------
while robot.step(TIME_STEP) != -1:
    t = paso * (TIME_STEP / 1000.0)

    # 1. PERCEPCIÓN: Lectura y pre-procesamiento de sensores
    ps0 = sensores["ps0"].getValue()
    ps7 = sensores["ps7"].getValue()
    ps1 = sensores["ps1"].getValue()
    ps2 = sensores["ps2"].getValue()
    ps5 = sensores["ps5"].getValue()
    ps6 = sensores["ps6"].getValue()

    dist_cruda   = (sensor_a_metros(ps0) + sensor_a_metros(ps7)) / 2.0
    dist_ema     = filtro_ema(dist_cruda, ema_anterior)
    ema_anterior = dist_ema
    
    pared_izq = max(ps5, ps6)
    pared_der = max(ps1, ps2)

    # 2. ESTIMACIÓN: Actualización de Odometría (Lab 1)
    act_izq = enc_izq.getValue()
    act_der = enc_der.getValue()
    d_izq   = encoder_a_lineal(act_izq - ant_enc_izq)
    d_der   = encoder_a_lineal(act_der - ant_enc_der)
    x_global, y_global, phi_global = calcular_odometria(x_global, y_global, phi_global, d_der, d_izq)
    
    ant_enc_izq = act_izq
    ant_enc_der = act_der

    # 3. FILTRADO: Fusión sensorial Odometría + IR con Kalman (Lab 2)
    kalman, _ = ejecutar_filtro_kalman((d_izq + d_der) / 2.0, dist_ema)

    # 4. CONTROL DE MOVIMIENTO Y NAVEGACIÓN
    vel_izq = vel_der = 0.0
    accion  = "DETENIDO"

    if idx_wp < len(ruta):
        mx, my  = ruta[idx_wp]
        ang_des = math.atan2(my - y_global, mx - x_global)
        err_ang = math.atan2(math.sin(ang_des - phi_global), math.cos(ang_des - phi_global))
        dist_wp = math.hypot(mx - x_global, my - y_global)

        # A. Condición de llegada al Waypoint (Tolerancia fina para evitar Corner Cutting)
        if dist_wp < 0.07:
            idx_wp += 1
            accion  = f"WP_{idx_wp}"
            print(f"Waypoint {idx_wp - 1} alcanzado")

        # B. Detección de colisión inminente (< 5cm)
        elif kalman <= 0.05 and contador_evasion == 0 and abs(err_ang) < 0.3:
            contador_evasion = 25 

        # C. Maniobra de Evasión Reactiva utilizando sensores laterales
        if contador_evasion > 0:
            if pared_izq > pared_der:
                vel_izq, vel_der = 4.0, -4.0
                accion = "EVADIENDO_DER"
            else:
                vel_izq, vel_der = -4.0, 4.0
                accion = "EVADIENDO_IZQ"
            contador_evasion -= 1

        # D. Corrección de rumbo (Giro sobre el eje hasta alinear el robot)
        elif abs(err_ang) > 0.02:
            w       = 3.0 * err_ang
            vel_izq = (-w * L / 2.0) / RADIO_RUEDA
            vel_der = ( w * L / 2.0) / RADIO_RUEDA
            accion  = "GIRANDO"

        # E. Movimiento rectilíneo hacia el objetivo
        else:
            vel_izq = vel_der = VELOCIDAD_AVANCE / RADIO_RUEDA
            accion  = "AVANZAR"

    else:
        # Fin de la ruta planificada
        if not ya_celebro:
            print("=" * 40)
            print(f"Misión Cumplida: Ruta en Escenario {ESCENARIO} finalizada exitosamente.")
            print("=" * 40)
            ya_celebro = True
        accion = "COMPLETADO"

    # 5. APLICAR POTENCIA A LOS MOTORES
    vel_izq = max(min(vel_izq, MAX_VELOCIDAD), -MAX_VELOCIDAD)
    vel_der = max(min(vel_der, MAX_VELOCIDAD), -MAX_VELOCIDAD)
    motor_izq.setVelocity(vel_izq)
    motor_der.setVelocity(vel_der)

    # 6. REGISTRO DE DATOS Y TELEMETRÍA
    guardar_csv(escritor, paso, t, x_global, y_global, math.degrees(phi_global), accion)

    # Imprimir estado general en consola 1 vez por segundo simulado
    if paso % int(1000 / TIME_STEP) == 0:
        print(f"Pose -> X:{x_global:.2f} Y:{y_global:.2f} Ang:{math.degrees(phi_global):.0f}° | Estado: {accion}")

    paso += 1

# Cierre seguro de archivos al terminar
archivo.close()