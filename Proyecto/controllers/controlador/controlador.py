import math
import os
import heapq
import csv
import time
from controller import Robot

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
    open_list = []
    closed_set = set()

    start_node = Node(start)
    end_node = Node(end)
    heapq.heappush(open_list, start_node)
    
    nodes_expanded = 0

    while open_list:
        current = heapq.heappop(open_list)
        nodes_expanded += 1

        if current.position == end_node.position:
            path = []
            node = current
            while node:
                path.append(node.position)
                node = node.parent
            return path[::-1], nodes_expanded

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

            dominated = False
            for on in open_list:
                if on == neighbor and on.g <= neighbor.g:
                    dominated = True
                    break
            if not dominated:
                heapq.heappush(open_list, neighbor)

    return None, nodes_expanded


def simplify_path(path):
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    for i in range(1, len(path) - 1):
        pr, pc = path[i - 1]
        cr, cc = path[i]
        nr, nc = path[i + 1]
        if (nr - pr, nc - pc) != (2 * (cr - pr), 2 * (cc - pc)):
            simplified.append(path[i])
    simplified.append(path[-1])
    return simplified


def main():
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())

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
        map_path = os.path.join(os.path.dirname(__file__), "maze_map.txt")
        if not os.path.exists(map_path):
            print(f"ERROR: No se encontró el archivo de mapa en: {map_path}")
            return
        
    with open(map_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    maze = [list(line.strip()) for line in lines if line.strip()]

    rows = len(maze)
    cols = len(maze[0])
    print(f"[MAPA] Cargado mapa '{map_filename}' de dimensiones {rows}x{cols}:")
    for row in maze:
        print("  " + " ".join(row))

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

    raw_path_result = astar(maze, start, end)
    
    if raw_path_result[0] is None:
        print("ERROR: No existe ruta posible en el mapa.")
        return
        
    raw_path, nodes_expanded = raw_path_result
    path_rc = simplify_path(raw_path)

    CELL = 0.25
    rows = len(maze)
    cols = len(maze[0])
    off_x = -cols * CELL / 2.0 + CELL / 2.0
    off_y = -rows * CELL / 2.0 + CELL / 2.0

    waypoints = []
    for (r, c) in path_rc:
        wx = off_x + c * CELL
        wy = -(off_y + r * CELL)
        waypoints.append((wx, wy))

    planned_dist = 0.0
    for i in range(len(waypoints) - 1):
        planned_dist += math.hypot(waypoints[i+1][0] - waypoints[i][0], waypoints[i+1][1] - waypoints[i][1])

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

    WHEEL_RADIUS = 0.0205
    AXLE_LENGTH  = 0.052
    MAX_SPEED    = 6.28

    V_FORWARD = 0.04
    OMEGA_TURN = 1.2

    ARRIVAL_DIST = 0.04

    pose_x, pose_y = waypoints[0]
    pose_theta = 0.0

    for _ in range(20):
        robot.step(timestep)
    prev_left  = left_enc.getValue()
    prev_right = right_enc.getValue()

    EMA_ALPHA  = 0.5
    ps_ema = [0.0] * 8

    target_idx = 1

    accumulated_dist = 0.0
    collision_events = 0
    in_collision = False
    log_data = []
    step_count = 0
    prev_state = None

    dist = 0.0
    current_time = 0.0

    while robot.step(timestep) != -1:
        current_time = step_count * (timestep / 1000.0)

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

        accumulated_dist += abs(dc)

        for i in range(8):
            raw = ps_sensors[i].getValue()
            ps_ema[i] = EMA_ALPHA * raw + (1.0 - EMA_ALPHA) * ps_ema[i]

        is_colliding_now = any(val > 800.0 for val in ps_ema)
        if is_colliding_now:
            if not in_collision:
                collision_events += 1
                in_collision = True
        else:
            in_collision = False

        if target_idx >= len(waypoints):
            left_motor.setVelocity(0.0)
            right_motor.setVelocity(0.0)
            print(f"\n[ÉXITO - {current_time:.2f}s] ¡META ALCANZADA CON ÉXITO!")
            break

        tx, ty = waypoints[target_idx]
        dx = tx - pose_x
        dy = ty - pose_y
        dist = math.hypot(dx, dy)

        if dist < ARRIVAL_DIST:
            target_idx += 1
            continue

        target_ang  = math.atan2(dy, dx)
        angle_err   = math.atan2(math.sin(target_ang - pose_theta),
                                 math.cos(target_ang - pose_theta))

        is_turning = False
        if abs(angle_err) > 0.175:
            omega = OMEGA_TURN * angle_err
            v     = 0.0
            is_turning = True
        else:
            omega = OMEGA_TURN * angle_err
            v     = V_FORWARD

        is_evading = False
        if v > 0.0 and (ps_ema[0] > 200 and ps_ema[7] > 200):
            v     = 0.0
            omega = OMEGA_TURN * angle_err * 2.0
            is_evading = True

        if is_evading:
            current_state = "EVADING"
        elif is_turning:
            current_state = "TURNING"
        else:
            current_state = "DRIVING"

        prev_state = current_state

        left_spd  = (v - omega * AXLE_LENGTH / 2.0) / WHEEL_RADIUS
        right_spd = (v + omega * AXLE_LENGTH / 2.0) / WHEEL_RADIUS

        left_spd  = max(min(left_spd,  MAX_SPEED), -MAX_SPEED)
        right_spd = max(min(right_spd, MAX_SPEED), -MAX_SPEED)

        left_motor.setVelocity(left_spd)
        right_motor.setVelocity(right_spd)

        if step_count % 30 == 0:
            print(f"[Estado {current_time:.1f}s] Pos: ({pose_x:.3f}, {pose_y:.3f}) | Obj: ({tx:.3f}, {ty:.3f}) | Dist: {dist:.3f}m | AngErr: {math.degrees(angle_err):.1f}° | Colisiones: {collision_events}")

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

    log_dir = os.path.dirname(__file__)
    csv_path = os.path.join(log_dir, f"trajectory_log_{world_name}.csv")
    summary_path = os.path.join(log_dir, f"summary_{world_name}.txt")

    try:
        with open(csv_path, "w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(["time_s", "x_m", "y_m", "theta_rad", "target_x_m", "target_y_m", "distance_to_target_m", "accumulated_distance_m", "is_colliding"])
            writer.writerows(log_data)
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

        print("\n" + "="*70)
        print(f"          RESUMEN DE EVALUACIÓN EXPERIMENTAL: {world_name.upper()}          ")
        print("="*70)
        print(f"  Tiempo total simulado:             {current_time:.2f} s")
        print(f"  Longitud de ruta planificada:      {planned_dist:.4f} m")
        print(f"  Longitud de trayectoria real:      {accumulated_dist:.4f} m")
        deviation = accumulated_dist - planned_dist
        pct_deviation = (deviation / planned_dist * 100) if planned_dist > 0 else 0.0
        print(f"  Desviación (Real - Planificada):   {deviation:+.4f} m ({pct_deviation:+.1f}%)")
        print(f"  Número de colisiones/roces:        {collision_events}")
        print(f"  Error de posición final a la meta: {dist:.4f} m")
        print("="*70 + "\n")
    except Exception as e:
        print(f"Error escribiendo resumen: {e}")


if __name__ == "__main__":
    main()
