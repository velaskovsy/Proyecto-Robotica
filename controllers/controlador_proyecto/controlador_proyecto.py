"""Controlador Proyecto Final: A* + Odometría + Kalman (Línea A)"""
import math
import csv
import heapq
import time
import os
from controller import Robot

# CONFIGURACIÓN Y CONSTANTES DEL ROBOT 
RADIO_RUEDA = 0.0205
L = 0.052  # Distancia entre las ruedas (De tu Lab 1)
MAX_VELOCIDAD = 6.28

# Parámetros matemáticos del Filtro de Kalman (De tu Lab 2)
Q_PROCESO = 0.01
R_MEDICION = 0.4
ALPHA_EMA = 0.3

# Variables globales para guardar el estado del Filtro de Kalman
kalman_distancia = 1.0
kalman_incertidumbre = 1.0
kalman_prediccion = 1.0


# MATEMÁTICAS, FILTROS Y ODOMETRÍA. Acá entra el contenido del laboratorio 2
def sensor_a_metros(valor_crudo):
    valor_crudo = max(valor_crudo, 1.0)
    return (0.05 * 1024.0) / valor_crudo


def encoder_a_lineal(angulo_delta):
    return RADIO_RUEDA * angulo_delta


def filtro_promedio_movil(valor_nuevo, valor_anterior):
    return ALPHA_EMA * valor_nuevo + (1 - ALPHA_EMA) * valor_anterior


def ejecutar_filtro_kalman(avance_robot, medicion_sensor):
    global kalman_distancia, kalman_incertidumbre, kalman_prediccion

    # 1. Predicción
    kalman_prediccion = kalman_distancia - avance_robot
    kalman_incertidumbre = kalman_incertidumbre + Q_PROCESO

    # 2. Corrección
    ganancia_bucle = kalman_incertidumbre / (kalman_incertidumbre + R_MEDICION)
    kalman_distancia = kalman_prediccion + ganancia_bucle * (medicion_sensor - kalman_prediccion)
    kalman_incertidumbre = (1 - ganancia_bucle) * kalman_incertidumbre

    return kalman_distancia, ganancia_bucle


def calcular_odometria(x_ant, y_ant, phi_ant, delta_s_der, delta_s_izq):
    """Calcula la nueva posición basada en la cinemática diferencial."""
    delta_s = (delta_s_der + delta_s_izq) / 2.0
    delta_phi = (delta_s_der - delta_s_izq) / L
    
    x_nuevo = x_ant + delta_s * math.cos(phi_ant + (delta_phi / 2.0))
    y_nuevo = y_ant + delta_s * math.sin(phi_ant + (delta_phi / 2.0))
    phi_nuevo = phi_ant + delta_phi
    
    phi_nuevo = math.atan2(math.sin(phi_nuevo), math.cos(phi_nuevo))
    return x_nuevo, y_nuevo, phi_nuevo


# Se crea el archivo csv con tus columnas de trayectoria requeridas
def crear_archivo_csv(nombre_archivo="datos_trayectoria_escenario1.csv"):
    ruta = os.path.join(os.path.dirname(__file__), nombre_archivo)
    archivo = open(ruta, "w", newline="")
    escritor = csv.writer(archivo)
    escritor.writerow(["step", "time_s", "x_robot", "y_robot", "angulo_grados", "action"])
    return archivo, escritor


def guardar_fila_csv(escritor, paso, tiempo, x, y, angulo, accion):
    escritor.writerow([
        paso, round(tiempo, 3), round(x, 4), round(y, 4), round(angulo, 2), accion
    ])

# PLANIFICACIÓN DE RUTA CON A*
MAPA_GRILLA = [
    [0, 0, 1, 0],  # Fila 0: El primer elemento es el Inicio [0,0]
    [0, 0, 1, 0], 
    [1, 0, 0, 0], 
    [0, 1, 0, 0]   # Fila 3: El último elemento es la Meta [3,3]
]

TAMANIO_CELDA = 0.25  # Mapa 1x1 metro / 4 celdas = 0.25 m por celda

def heuristica(a, b):
    """Distancia Manhattan entre dos celdas."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def planificar_ruta_a_star(inicio, meta, mapa):
    """Encuentra la ruta óptima en la grilla usando A* y aplica el Offset."""
    filas = len(mapa)
    cols  = len(mapa[0])

    heap = []
    heapq.heappush(heap, (0, inicio))
    costo_g   = {inicio: 0}
    came_from = {inicio: None}
    direcciones = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while heap:
        _, actual = heapq.heappop(heap)

        if actual == meta:
            camino_celdas = []
            nodo = meta
            while nodo is not None:
                camino_celdas.append(nodo)
                nodo = came_from[nodo]
            camino_celdas.reverse()

            ruta_metros = []
            for (fila, col) in camino_celdas[1:]:
                # Conversión con Offset para el mundo centrado de Webots
                x = (col * 0.25) - 0.375
                y = 0.375 - (fila * 0.25)
                ruta_metros.append((x, y))

            print(f"Ruta A* encontrada: {len(ruta_metros)} puntos intermedios")
            print(f"Celdas: {camino_celdas}")
            print(f"Metros: {ruta_metros}")
            return ruta_metros

        for df, dc in direcciones:
            vecino = (actual[0] + df, actual[1] + dc)
            fv, cv = vecino
            if not (0 <= fv < filas and 0 <= cv < cols):
                continue
            if mapa[fv][cv] == 1:
                continue
            nuevo_g = costo_g[actual] + 1
            if vecino not in costo_g or nuevo_g < costo_g[vecino]:
                costo_g[vecino]   = nuevo_g
                f_costo           = nuevo_g + heuristica(vecino, meta)
                came_from[vecino] = actual
                heapq.heappush(heap, (f_costo, vecino))

    print("¡Error! No existe ruta entre inicio y meta.")
    return []


# BLOQUE 4: INICIALIZACIÓN DE WEBOTS 
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())

# Configurar motores estilo Lab 2
motor_izquierdo = robot.getDevice("left wheel motor")
motor_derecho = robot.getDevice("right wheel motor")
motor_izquierdo.setPosition(float("inf"))
motor_derecho.setPosition(float("inf"))
motor_izquierdo.setVelocity(0.0)
motor_derecho.setVelocity(0.0)

encoder_izquierdo = robot.getDevice("left wheel sensor")
encoder_derecho = robot.getDevice("right wheel sensor")
encoder_izquierdo.enable(TIME_STEP)
encoder_derecho.enable(TIME_STEP)

# Inicializar tus sensores mediante el mapeo por diccionario del Lab 2
nombres_sensores = ["ps0", "ps1", "ps2", "ps5", "ps6", "ps7"]
sensores = {}
for nombre in nombres_sensores:
    dispositivo = robot.getDevice(nombre)
    dispositivo.enable(TIME_STEP)
    sensores[nombre] = dispositivo

# Configuración de odometría inicial con Offset (Mirando al Sur)
x_global = (0 * 0.25) - 0.375
y_global = 0.375 - (0 * 0.25)
phi_global = -math.pi / 2 

# Planificar ruta de celda (0,0) a celda (3,3)
ruta_planificada = planificar_ruta_a_star((0, 0), (3, 3), MAPA_GRILLA)
indice_meta_actual = 0

# Archivo de registro de datos 
archivo_csv, escritor_csv = crear_archivo_csv()

paso_actual = 0
contador_evasion = 0
ya_celebro = False

# Primer paso para estabilizar encoders anteriores
robot.step(TIME_STEP)
anterior_enc_izq = encoder_izquierdo.getValue()
anterior_enc_der = encoder_derecho.getValue()

print("Comenzando navegación y grabación de datos...")

# --------------------- MAIN --------------------------
while robot.step(TIME_STEP) != -1:
    tiempo_segundos = paso_actual * (TIME_STEP / 1000.0)

    # 1. Leer sensores mediante tu diccionario (Estilo Lab 2)
    sensor_ps0 = sensores["ps0"].getValue()
    sensor_ps7 = sensores["ps7"].getValue()
    
    # Convertir sensores frontales a metros y promediar
    distancia_frontal_cruda = (sensor_a_metros(sensor_ps0) + sensor_a_metros(sensor_ps7)) / 2.0

    # 2. Calcular odometría usando tus variables y la función lineal
    actual_enc_izq = encoder_izquierdo.getValue()
    actual_enc_der = encoder_derecho.getValue()

    giro_izquierdo = actual_enc_izq - anterior_enc_izq
    giro_derecho = actual_enc_der - anterior_enc_der

    delta_s_izq = encoder_a_lineal(giro_izquierdo)
    delta_s_der = encoder_a_lineal(giro_derecho)

    x_global, y_global, phi_global = calcular_odometria(
        x_global, y_global, phi_global, delta_s_der, delta_s_izq
    )

    anterior_enc_izq = actual_enc_izq
    anterior_enc_der = actual_enc_der

    # 3. Filtro de Kalman 
    distancia_avanzada = (delta_s_izq + delta_s_der) / 2.0
    estimacion_kalman, ganancia_kalman = ejecutar_filtro_kalman(distancia_avanzada, distancia_frontal_cruda)

    # 4. LÓGICA DE MOVIMIENTO INTEGRADA
    vel_izq = 0.0
    vel_der = 0.0
    accion_actual = "DETENIDO"

    if indice_meta_actual < len(ruta_planificada):
        meta_x, meta_y = ruta_planificada[indice_meta_actual]

        angulo_deseado = math.atan2(meta_y - y_global, meta_x - x_global)
        error_angulo = math.atan2(
            math.sin(angulo_deseado - phi_global),
            math.cos(angulo_deseado - phi_global)
        )
        distancia_a_meta = math.hypot(meta_x - x_global, meta_y - y_global)

        # Verificar si alcanzamos el punto intermedio de la ruta
        if distancia_a_meta < 0.05:
            indice_meta_actual += 1
            accion_actual = f"WP_ALCANZADO_{indice_meta_actual}"
            print(f"¡Waypoint {indice_meta_actual - 1} alcanzado!")

        # Bloqueador de evasión reactiva ante emergencias reales (Umbral 5cm)
        elif estimacion_kalman < 0.05 and contador_evasion == 0 and abs(error_angulo) < 0.3:
            contador_evasion = 25
            accion_actual = "INICIO_EVASION"

        if contador_evasion > 0:
            vel_izq = -4.0
            vel_der = 4.0
            contador_evasion -= 1
            accion_actual = "EVADIENDO"

        elif abs(error_angulo) > 0.05: 
            # GIRANDO EN EL LUGAR (Cinemática inversa usando L de tu Lab 1)
            v_lineal = 0.0
            w_angular = 2.0 * error_angulo 
            vel_izq = (v_lineal - (w_angular * L / 2.0)) / RADIO_RUEDA
            vel_der = (v_lineal + (w_angular * L / 2.0)) / RADIO_RUEDA
            accion_actual = "GIRANDO"

        else:
            # AVANZAR hacia la meta con tracción y velocidad estable
            v_lineal = 0.08 
            w_angular = 0.0
            vel_izq = (v_lineal - (w_angular * L / 2.0)) / RADIO_RUEDA
            vel_der = (v_lineal + (w_angular * L / 2.0)) / RADIO_RUEDA
            accion_actual = "AVANZAR"

    else:
        if not ya_celebro: 
            print("=" * 40)
            print("¡Ruta Completada! El robot llegó con éxito a la meta.")
            print("=" * 40)
            ya_celebro = True

        vel_izq = 0.0
        vel_der = 0.0
        accion_actual = "RUTA_COMPLETADA"

    # Asegurar límites físicos de los motores del e-puck
    vel_izq = max(min(vel_izq, MAX_VELOCIDAD), -MAX_VELOCIDAD)
    vel_der = max(min(vel_der, MAX_VELOCIDAD), -MAX_VELOCIDAD)
    motor_izquierdo.setVelocity(vel_izq)
    motor_derecho.setVelocity(vel_der)

    # 5. Guardar fila en el CSV de telemetría usando tu función original
    guardar_fila_csv(escritor_csv, paso_actual, tiempo_segundos, x_global, y_global, math.degrees(phi_global), accion_actual)

    # Impresión de estado limpia una vez por segundo
    if paso_actual % int(1000 / TIME_STEP) == 0:
        print(f"Pose -> X:{x_global:.2f} Y:{y_global:.2f} Ang:{math.degrees(phi_global):.0f}° | Estado: {accion_actual}")

    paso_actual += 1

# Cierre del documento
archivo_csv.close()
print("Se exportó el archivo con los datos de trayectoria de forma exitosa.")