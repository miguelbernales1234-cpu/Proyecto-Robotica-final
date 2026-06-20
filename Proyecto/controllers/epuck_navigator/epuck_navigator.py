import math
import os
import heapq
import csv
from controller import Robot

# =============================================================================
# MÓDULO A*: Planificación global de ruta
# =============================================================================
class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __lt__(self, other):
        return self.f < other.f


def astar(maze, start, end):
    """
    Algoritmo A* sobre una grilla 2D.
    Devuelve la lista de celdas (fila, col) desde start hasta end.
    """
    open_list = []
    closed_set = set()

    start_node = Node(start)
    end_node = Node(end)
    heapq.heappush(open_list, start_node)

    while open_list:
        current = heapq.heappop(open_list)

        if current.position == end_node.position:
            path = []
            node = current
            while node:
                path.append(node.position)
                node = node.parent
            return path[::-1]

        closed_set.add(current.position)

        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nr, nc = current.position[0] + dr, current.position[1] + dc

            if nr < 0 or nr >= len(maze) or nc < 0 or nc >= len(maze[0]):
                continue
            if maze[nr][nc] == '#':
                continue
            if (nr, nc) in closed_set:
                continue

            neighbor = Node((nr, nc), current)
            neighbor.g = current.g + 1
            neighbor.h = abs(nr - end_node.position[0]) + abs(nc - end_node.position[1])
            neighbor.f = neighbor.g + neighbor.h

            # Solo agregar si no hay ya un nodo mejor en open_list
            dominated = False
            for on in open_list:
                if on == neighbor and on.g <= neighbor.g:
                    dominated = True
                    break
            if not dominated:
                heapq.heappush(open_list, neighbor)

    return None


# =============================================================================
# MÓDULO WAYPOINT: Simplificación de ruta (elimina nodos intermedios colineales)
# =============================================================================
def simplify_path(path):
    """
    Elimina puntos intermedios en segmentos rectos para tener menos waypoints.
    Esto reduce los errores de dirección al tener que girar menos veces.
    """
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    for i in range(1, len(path) - 1):
        pr, pc = path[i - 1]
        cr, cc = path[i]
        nr, nc = path[i + 1]
        # Si el punto actual NO está en línea recta entre el anterior y el siguiente, es una esquina
        if (nr - pr, nc - pc) != (2 * (cr - pr), 2 * (cc - pc)):
            simplified.append(path[i])
    simplified.append(path[-1])
    return simplified


# =============================================================================
# FUNCIÓN PRINCIPAL
# =============================================================================
def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

    # ------------------------------------------------------------------
    # 1. Cargar mapa y ejecutar A* dinámicamente según el mundo
    # ------------------------------------------------------------------
    try:
        world_path = robot.getWorldPath()
        world_name = os.path.splitext(os.path.basename(world_path))[0]
    except Exception:
        world_name = "complejo"

    if "simple" in world_name.lower():
        map_filename = "maze_map_simple.txt"
    elif "complejo" in world_name.lower():
        map_filename = "maze_map_complejo.txt"
    else:
        map_filename = "maze_map.txt"

    map_path = os.path.join(os.path.dirname(__file__), map_filename)
    if not os.path.exists(map_path):
        # Fallback si no existe el renombrado
        map_path = os.path.join(os.path.dirname(__file__), "maze_map.txt")
        if not os.path.exists(map_path):
            print(f"ERROR: No se encontró el archivo de mapa en: {map_path}")
            return
        
    with open(map_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    maze = [list(line.strip()) for line in lines if line.strip()]

    start = end = None
    for r, row in enumerate(maze):
        for c, cell in enumerate(row):
            if cell == 'S':
                start = (r, c)
            elif cell == 'E':
                end = (r, c)

    if start is None or end is None:
        print("ERROR: No se encontraron los puntos de inicio (S) o fin (E) en el mapa")
        return

    print(f"[{world_name}] A*: Buscando ruta desde el inicio {start} hasta la meta {end}...")
    raw_path = astar(maze, start, end)
    if raw_path is None:
        print("ERROR: No existe ruta posible en el mapa.")
        return

    path_rc = simplify_path(raw_path)
    print(f"Ruta encontrada: {len(raw_path)} celdas → {len(path_rc)} waypoints tras simplificación.")

    # ------------------------------------------------------------------
    # 2. Convertir celdas (fila, col) → coordenadas mundo (x, y) en metros
    # ------------------------------------------------------------------
    CELL = 0.25          # Tamaño de celda en metros (coincide con generate_maze.py)
    rows = len(maze)
    cols = len(maze[0])
    off_x = -cols * CELL / 2.0 + CELL / 2.0
    off_y = -rows * CELL / 2.0 + CELL / 2.0

    waypoints = []
    for (r, c) in path_rc:
        wx = off_x + c * CELL
        wy = -(off_y + r * CELL)
        waypoints.append((wx, wy))

    # Calcular distancia planificada total en metros
    planned_dist = 0.0
    for i in range(len(waypoints) - 1):
        planned_dist += math.hypot(waypoints[i+1][0] - waypoints[i][0], waypoints[i+1][1] - waypoints[i][1])

    # ------------------------------------------------------------------
    # 3. Inicializar dispositivos
    # ------------------------------------------------------------------
    left_motor  = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)

    ps_sensors = []
    for name in ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']:
        s = robot.getDevice(name)
        s.enable(timestep)
        ps_sensors.append(s)

    left_enc  = robot.getDevice('left wheel sensor')
    right_enc = robot.getDevice('right wheel sensor')
    left_enc.enable(timestep)
    right_enc.enable(timestep)

    # ------------------------------------------------------------------
    # 4. Constantes del E-puck
    # ------------------------------------------------------------------
    WHEEL_RADIUS = 0.0205   # metros
    AXLE_LENGTH  = 0.052    # metros (distancia entre ruedas)
    MAX_SPEED    = 6.28     # rad/s

    # Velocidades de navegación nominal (lentas para reducir patinaje)
    V_FORWARD = 0.04        # m/s velocidad lineal al avanzar
    OMEGA_TURN = 1.2        # ganancia proporcional del controlador de ángulo

    # Umbral de llegada al waypoint
    ARRIVAL_DIST = 0.04     # metros

    # ------------------------------------------------------------------
    # 5. Estado inicial del robot (odometría)
    # ------------------------------------------------------------------
    pose_x, pose_y = waypoints[0]
    pose_theta = 0.0  # Orientación inicial: mirando al eje +X (este)

    # Esperar unos steps para estabilizar encoders
    for _ in range(20):
        robot.step(timestep)
    prev_left  = left_enc.getValue()
    prev_right = right_enc.getValue()

    # Filtro EMA para sensores de distancia (α = 0.5)
    EMA_ALPHA  = 0.5
    ps_ema = [0.0] * 8

    target_idx = 1  # Empezamos apuntando al waypoint 1 (el 0 es donde estamos)

    # Variables de registro experimental
    accumulated_dist = 0.0
    collision_events = 0
    in_collision = False
    log_data = []
    step_count = 0

    print("Iniciando navegación autónoma...")

    # ------------------------------------------------------------------
    # 6. Bucle principal
    # ------------------------------------------------------------------
    dist = 0.0
    current_time = 0.0

    while robot.step(timestep) != -1:

        # ── A. ODOMETRÍA ────────────────────────────────────────────────
        cur_left  = left_enc.getValue()
        cur_right = right_enc.getValue()

        dl = (cur_left  - prev_left)  * WHEEL_RADIUS
        dr = (cur_right - prev_right) * WHEEL_RADIUS

        dc    = (dl + dr) / 2.0
        dphi  = (dr - dl) / AXLE_LENGTH

        pose_x     += dc * math.cos(pose_theta + dphi / 2.0)
        pose_y     += dc * math.sin(pose_theta + dphi / 2.0)
        pose_theta += dphi
        pose_theta  = math.atan2(math.sin(pose_theta), math.cos(pose_theta))

        prev_left  = cur_left
        prev_right = cur_right

        # Acumular distancia real recorrida
        accumulated_dist += abs(dc)

        # ── B. FILTRADO SENSORIAL (EMA) ──────────────────────────────────
        for i in range(8):
            raw = ps_sensors[i].getValue()
            ps_ema[i] = EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * ps_ema[i]

        # ── C. DETECCIÓN DE COLISIONES O CASI-COLISIONES ──────────────────
        # Si algún sensor supera 800 (obstáculo muy cercano), contamos una colisión
        is_colliding_now = any(val > 800.0 for val in ps_ema)
        if is_colliding_now:
            if not in_collision:
                collision_events += 1
                in_collision = True
        else:
            in_collision = False

        # ── D. LÓGICA DE NAVEGACIÓN ──────────────────────────────────────
        if target_idx >= len(waypoints):
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print("¡Meta alcanzada con éxito!")
            break

        tx, ty = waypoints[target_idx]
        dx = tx - pose_x
        dy = ty - pose_y
        dist = math.hypot(dx, dy)

        # Llegamos al waypoint actual → avanzar al siguiente
        if dist < ARRIVAL_DIST:
            target_idx += 1
            print(f"Waypoint {target_idx-1} alcanzado. Apuntando al waypoint {target_idx} de {len(waypoints)-1}...")
            continue

        # Ángulo hacia el objetivo
        target_ang  = math.atan2(dy, dx)
        angle_err   = math.atan2(math.sin(target_ang - pose_theta),
                                 math.cos(target_ang - pose_theta))

        # ── FASE 1: Giro en sitio hasta estar bien alineado ──
        # Solo avanzamos cuando el error de ángulo es menor a ~10 grados
        if abs(angle_err) > 0.175:   # > ~10 grados → solo girar
            omega = OMEGA_TURN * angle_err
            v     = 0.0
        else:                         # ≤ 10 grados → avanzar con corrección suave
            omega = OMEGA_TURN * angle_err
            v     = V_FORWARD

        # ── FASE 2: EVASIÓN DE EMERGENCIA (solo pared FRONTAL) ───────────
        # Si la pared está justo en frente, frenamos y rotamos en el sitio para no chocar.
        if v > 0.0 and (ps_ema[0] > 200 and ps_ema[7] > 200):
            v     = 0.0
            omega = OMEGA_TURN * angle_err * 2.0

        # ── CINEMÁTICA INVERSA ───────────────────────────────────────────
        left_spd  = (v - omega * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
        right_spd = (v + omega * AXLE_LENGTH / 2.0) / WHEEL_RADIUS

        # Acotar velocidades
        left_spd  = max(min(left_spd,  MAX_SPEED), -MAX_SPEED)
        right_spd = max(min(right_spd, MAX_SPEED), -MAX_SPEED)

        left_motor.setVelocity(left_spd)
        right_motor.setVelocity(right_spd)

        # Registrar métricas en este paso
        current_time = step_count * (timestep / 1000.0)
        log_data.append([
            f"{current_time:.2f}",
            f"{pose_x:.4f}",
            f"{pose_y:.4f}",
            f"{pose_theta:.4f}",
            f"{tx:.4f}",
            f"{ty:.4f}",
            f"{dist:.4f}",
            f"{accumulated_dist:.4f}",
            1 if is_colliding_now else 0
        ])
        
        step_count += 1

    # ------------------------------------------------------------------
    # 7. Guardar logs y resúmenes al finalizar
    # ------------------------------------------------------------------
    log_dir = os.path.dirname(__file__)
    csv_path = os.path.join(log_dir, f"trajectory_log_{world_name}.csv")
    summary_path = os.path.join(log_dir, f"summary_{world_name}.txt")

    print(f"Guardando datos experimentales para escenario: {world_name}...")

    try:
        with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["time_s", "x_m", "y_m", "theta_rad", "target_x_m", "target_y_m", "distance_to_target_m", "accumulated_distance_m", "is_colliding"])
            writer.writerows(log_data)
        print(f"CSV guardado con éxito en: {csv_path}")
    except Exception as e:
        print(f"Error escribiendo CSV: {e}")

    try:
        with open(summary_path, "w", encoding="utf-8") as f:
            f.write(f"=== RESUMEN DE EVALUACION EXPERIMENTAL: {world_name.upper()} ===\n")
            f.write(f"Tiempo total hasta meta: {current_time:.2f} s\n")
            f.write(f"Longitud de ruta planificada: {planned_dist:.4f} m\n")
            f.write(f"Longitud de trayectoria ejecutada: {accumulated_dist:.4f} m\n")
            f.write(f"Diferencia (Real - Planificada): {abs(accumulated_dist - planned_dist):.4f} m\n")
            f.write(f"Numero de colisiones o casi-colisiones detectadas: {collision_events}\n")
            f.write(f"Error de posicion final a la meta: {dist:.4f} m\n")
        print(f"Resumen de evaluación guardado con éxito en: {summary_path}")
    except Exception as e:
        print(f"Error escribiendo resumen: {e}")


if __name__ == "__main__":
    main()
