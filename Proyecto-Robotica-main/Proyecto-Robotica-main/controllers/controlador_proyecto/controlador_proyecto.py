"""Controlador Proyecto Final: A* + Odometría + Kalman (Línea A)"""
import math
import csv
import heapq
import os
from controller import Robot

# 1 -> Abrir mundo_proyecto.wbt  (arena 1x1, grilla 4x4)
# 2 -> Abrir Escenario_Complejo.wbt (arena 2x2, grilla 8x8)
ESCENARIO = 2

RADIO_RUEDA   = 0.0205
L             = 0.052
MAX_VELOCIDAD = 6.28
Q_PROCESO     = 0.01
R_MEDICION    = 0.4
ALPHA_EMA     = 0.3

kalman_distancia     = 1.0
kalman_incertidumbre = 1.0
kalman_prediccion    = 1.0

MAPA_E1 = [
    [0, 0, 1, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 0],
    [0, 1, 0, 0]
]

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

if ESCENARIO == 1:
    MAPA          = MAPA_E1
    TAMANIO_CELDA = 0.25
    OFFSET        = 0.375
    INICIO        = (0, 0)
    META          = (3, 3)
else:
    MAPA          = MAPA_E2
    TAMANIO_CELDA = 0.25
    OFFSET        = 0.875
    INICIO        = (0, 0)
    META          = (7, 7)

UMBRAL_OBSTACULO = 0.15
UMBRAL_LATERAL   = 80.0
VELOCIDAD_AVANCE = 0.08


def sensor_a_metros(v):
    return (0.05 * 1024.0) / max(v, 1.0)

def encoder_a_lineal(d):
    return RADIO_RUEDA * d

def filtro_ema(nuevo, anterior):
    return ALPHA_EMA * nuevo + (1 - ALPHA_EMA) * anterior

def ejecutar_filtro_kalman(avance, medicion):
    global kalman_distancia, kalman_incertidumbre, kalman_prediccion
    kalman_prediccion    = kalman_distancia - avance
    kalman_incertidumbre = kalman_incertidumbre + Q_PROCESO
    ganancia             = kalman_incertidumbre / (kalman_incertidumbre + R_MEDICION)
    kalman_distancia     = kalman_prediccion + ganancia * (medicion - kalman_prediccion)
    kalman_incertidumbre = (1 - ganancia) * kalman_incertidumbre
    return kalman_distancia, ganancia

def calcular_odometria(x, y, phi, d_der, d_izq):
    ds   = (d_der + d_izq) / 2.0
    dphi = (d_der - d_izq) / L
    x    = x + ds * math.cos(phi + dphi / 2.0)
    y    = y + ds * math.sin(phi + dphi / 2.0)
    phi  = math.atan2(math.sin(phi + dphi), math.cos(phi + dphi))
    return x, y, phi

def crear_csv(nombre):
    ruta = os.path.join(os.path.dirname(__file__), nombre)
    f = open(ruta, "w", newline="")
    w = csv.writer(f)
    w.writerow(["step","time_s","x_robot","y_robot","angulo_grados","sensor_crudo","kalman","action"])
    return f, w

def guardar_csv(w, paso, t, x, y, ang, sensor, kalman, accion):
    w.writerow([paso, round(t,3), round(x,4), round(y,4), round(ang,2), round(sensor,4), round(kalman,4), accion])

def heuristica(a, b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def planificar_a_star(inicio, meta, mapa):
    filas, cols = len(mapa), len(mapa[0])
    heap = [(0, inicio)]
    costo_g   = {inicio: 0}
    came_from = {inicio: None}
    dirs = [(-1,0),(1,0),(0,-1),(0,1)]
    while heap:
        _, actual = heapq.heappop(heap)
        if actual == meta:
            camino = []
            n = meta
            while n: camino.append(n); n = came_from[n]
            camino.reverse()
            ruta = [((c*TAMANIO_CELDA)-OFFSET, OFFSET-(f*TAMANIO_CELDA)) for f,c in camino[1:]]
            print(f"Ruta A*: {len(ruta)} waypoints -> {camino}")
            return ruta
        for df,dc in dirs:
            v = (actual[0]+df, actual[1]+dc)
            if not (0<=v[0]<filas and 0<=v[1]<cols): continue
            if mapa[v[0]][v[1]] == 1: continue
            ng = costo_g[actual] + 1
            if v not in costo_g or ng < costo_g[v]:
                costo_g[v]   = ng
                came_from[v] = actual
                heapq.heappush(heap, (ng + heuristica(v, meta), v))
    print("Sin ruta.")
    return []


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

sensores = {}
for nombre in ["ps0","ps1","ps2","ps5","ps6","ps7"]:
    s = robot.getDevice(nombre)
    s.enable(TIME_STEP)
    sensores[nombre] = s

x_global   = (INICIO[1] * TAMANIO_CELDA) - OFFSET
y_global   = OFFSET - (INICIO[0] * TAMANIO_CELDA)
phi_global = -math.pi / 2

ruta              = planificar_a_star(INICIO, META, MAPA)
idx_wp            = 0
archivo, escritor = crear_csv(f"datos_trayectoria_escenario{ESCENARIO}.csv")

paso         = 0
ya_celebro   = False
ema_anterior = 1.0

robot.step(TIME_STEP)
ant_enc_izq = enc_izq.getValue()
ant_enc_der = enc_der.getValue()

print(f"Escenario {ESCENARIO} iniciado.")

while robot.step(TIME_STEP) != -1:
    t = paso * (TIME_STEP / 1000.0)

    ps0 = sensores["ps0"].getValue()
    ps7 = sensores["ps7"].getValue()
    ps1 = sensores["ps1"].getValue()
    ps2 = sensores["ps2"].getValue()
    ps5 = sensores["ps5"].getValue()
    ps6 = sensores["ps6"].getValue()

    dist_cruda   = (sensor_a_metros(ps0) + sensor_a_metros(ps7)) / 2.0
    dist_ema     = filtro_ema(dist_cruda, ema_anterior)
    ema_anterior = dist_ema
    kalman, _    = ejecutar_filtro_kalman((encoder_a_lineal(enc_izq.getValue() - ant_enc_izq) + encoder_a_lineal(enc_der.getValue() - ant_enc_der)) / 2.0, dist_ema)

    pared_izq = max(ps5, ps6)
    pared_der = max(ps1, ps2)

    act_izq = enc_izq.getValue()
    act_der = enc_der.getValue()
    d_izq   = encoder_a_lineal(act_izq - ant_enc_izq)
    d_der   = encoder_a_lineal(act_der - ant_enc_der)
    x_global, y_global, phi_global = calcular_odometria(x_global, y_global, phi_global, d_der, d_izq)
    ant_enc_izq = act_izq
    ant_enc_der = act_der

    vel_izq = vel_der = 0.0
    accion  = "DETENIDO"

    if idx_wp < len(ruta):
        mx, my  = ruta[idx_wp]
        ang_des = math.atan2(my - y_global, mx - x_global)
        err_ang = math.atan2(math.sin(ang_des - phi_global), math.cos(ang_des - phi_global))
        dist_wp = math.hypot(mx - x_global, my - y_global)

        if dist_wp < 0.10:
            idx_wp += 1
            accion  = f"WP_{idx_wp}"
            print(f"Waypoint {idx_wp} alcanzado")

        elif kalman <= UMBRAL_OBSTACULO:
            if pared_izq > pared_der:
                vel_izq =  2.5
                vel_der = -2.5
                accion  = "GIRAR_DERECHA"
            else:
                vel_izq = -2.5
                vel_der =  2.5
                accion  = "GIRAR_IZQUIERDA"

        elif pared_izq > UMBRAL_LATERAL:
            vel_izq = VELOCIDAD_AVANCE / RADIO_RUEDA
            vel_der = VELOCIDAD_AVANCE * 0.3 / RADIO_RUEDA
            accion  = "CURVA_DERECHA"

        elif pared_der > UMBRAL_LATERAL:
            vel_izq = VELOCIDAD_AVANCE * 0.3 / RADIO_RUEDA
            vel_der = VELOCIDAD_AVANCE / RADIO_RUEDA
            accion  = "CURVA_IZQUIERDA"

        elif abs(err_ang) > 0.15:
            w       = 2.0 * err_ang
            vel_izq = (-w * L / 2.0) / RADIO_RUEDA
            vel_der = ( w * L / 2.0) / RADIO_RUEDA
            accion  = "GIRANDO"

        else:
            vel_izq = vel_der = VELOCIDAD_AVANCE / RADIO_RUEDA
            accion  = "AVANZAR"

    else:
        if not ya_celebro:
            print(f"Ruta Escenario {ESCENARIO} completada.")
            ya_celebro = True
        accion = "COMPLETADO"

    vel_izq = max(min(vel_izq, MAX_VELOCIDAD), -MAX_VELOCIDAD)
    vel_der = max(min(vel_der, MAX_VELOCIDAD), -MAX_VELOCIDAD)
    motor_izq.setVelocity(vel_izq)
    motor_der.setVelocity(vel_der)

    guardar_csv(escritor, paso, t, x_global, y_global, math.degrees(phi_global), dist_cruda, kalman, accion)

    if paso % int(1000 / TIME_STEP) == 0:
        print(f"X:{x_global:.2f} Y:{y_global:.2f} Ang:{math.degrees(phi_global):.0f}° | {accion}")

    paso += 1

archivo.close()
print("CSV exportado.")