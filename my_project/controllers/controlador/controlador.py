import os
import math
import heapq
from controller import Robot

def a_estrella(grilla, inicio_fc, meta_fc):
    tamano = len(grilla)
    conjunto_abierto = []
    heapq.heappush(conjunto_abierto, (0, inicio_fc))
    proviene_de = {}
    g_score = {inicio_fc: 0}
    
    vecinos = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    
    while conjunto_abierto:
        actual = heapq.heappop(conjunto_abierto)[1]
        
        if actual == meta_fc:
            camino = []
            while actual in proviene_de:
                camino.append(actual)
                actual = proviene_de[actual]
            camino.append(inicio_fc)
            camino.reverse()
            return camino
            
        r, c = actual
        for dr, dc in vecinos:
            nr, nc = r + dr, c + dc
            if 0 <= nr < tamano and 0 <= nc < tamano and grilla[nr][nc] == 0:
                tentativo_g = g_score[actual] + 1
                if tentativo_g < g_score.get((nr, nc), float('inf')):
                    proviene_de[(nr, nc)] = actual
                    g_score[(nr, nc)] = tentativo_g
                    h = math.hypot(meta_fc[0] - nr, meta_fc[1] - nc)
                    heapq.heappush(conjunto_abierto, (tentativo_g + h, (nr, nc)))
    return None

def main():
    robot = Robot()
    paso_tiempo = int(robot.getBasicTimeStep())
    dt = paso_tiempo / 1000.0
    
    print("Inicializando dispositivos del robot...")
    motor_izquierdo = robot.getDevice("left wheel motor")
    motor_derecho = robot.getDevice("right wheel motor")
    motor_izquierdo.setPosition(float('inf'))
    motor_derecho.setPosition(float('inf'))
    motor_izquierdo.setVelocity(0.0)
    motor_derecho.setVelocity(0.0)
    
    encoder_izquierdo = robot.getDevice("left wheel sensor")
    encoder_derecho = robot.getDevice("right wheel sensor")
    encoder_izquierdo.enable(paso_tiempo)
    encoder_derecho.enable(paso_tiempo)
    
    gps = robot.getDevice("gps")
    gps.enable(paso_tiempo)
    
    imu = robot.getDevice("inertial unit")
    imu.enable(paso_tiempo)
    
    sensores_distancia = []
    for i in range(8):
        sensor = robot.getDevice(f"ps{i}")
        sensor.enable(paso_tiempo)
        sensores_distancia.append(sensor)
        
    for _ in range(5):
        robot.step(paso_tiempo)
        
    r_rueda = 0.0205
    L_eje = 0.052
    V_max = 0.08
    
    grilla = [
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1],
        [1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1],
        [1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1],
        [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1],
        [1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1],
        [1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1],
        [1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 1, 0, 1],
        [1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1],
        [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
    ]
    tamano_grilla = 13
    
    inicio_fc = (11, 1)
    meta_fc = (1, 11)
    
    print("Calculando ruta A*...")
    camino_grilla = a_estrella(grilla, inicio_fc, meta_fc)
    
    if camino_grilla is None:
        print("Error: ¡No se encontró ruta con A*!")
        puntos_ruta = []
    else:
        print(f"¡Ruta encontrada con {len(camino_grilla)} celdas!")
        puntos_ruta = []
        for fila, col in camino_grilla:
            x_w = -1.5 + col * 0.25
            y_w = 1.5 - fila * 0.25
            puntos_ruta.append((x_w, y_w))
        print("Puntos de ruta:", puntos_ruta)
        
    longitud_planificada = 0.0
    for i in range(1, len(puntos_ruta)):
        longitud_planificada += math.hypot(puntos_ruta[i][0] - puntos_ruta[i-1][0], puntos_ruta[i][1] - puntos_ruta[i-1][1])
    print(f"Longitud de ruta planificada: {longitud_planificada:.3f} m")

    pos_izq_ant = encoder_izquierdo.getValue()
    pos_der_ant = encoder_derecho.getValue()
    
    val_gps = gps.getValues()
    val_imu = imu.getRollPitchYaw()
    
    x_inicial = val_gps[0] if not math.isnan(val_gps[0]) else -1.25
    y_inicial = val_gps[1] if not math.isnan(val_gps[1]) else -1.25
    phi_inicial = val_imu[2] if not math.isnan(val_imu[2]) else 0.0
    
    x_odom, y_odom, phi_odom = x_inicial, y_inicial, phi_inicial
    x_est, y_est, phi_est = x_inicial, y_inicial, phi_inicial
    
    P_xx, P_yy, P_ff = 0.01, 0.01, 0.01
    Q_xx, Q_yy, Q_ff = 1e-5, 1e-5, 1e-4
    R_xx, R_yy, R_ff = 1e-4, 1e-4, 1e-5
         
    indice_punto = 0
    total_puntos = len(puntos_ruta)
    
    temporizador_atascamiento = 0.0
    ult_x_atascamiento = x_inicial
    ult_y_atascamiento = y_inicial
    tiempo_recuperacion_restante = 0.0
    v_recuperacion = 0.0
    omega_recuperacion = 0.0
    
    tray_real = []
    tray_odom = []
    tray_kalman = []
    
    tiempo_inicio = robot.getTime()
    conteo_colisiones = 0
    en_estado_colision = False
    
    longitud_tray_real = 0.0
    longitud_tray_odom = 0.0
    
    pos_real_ant = (x_inicial, y_inicial)
    pos_odom_ant = (x_inicial, y_inicial)
    
    while robot.step(paso_tiempo) != -1:
        tiempo_actual = robot.getTime()
        
        pos_izq = encoder_izquierdo.getValue()
        pos_der = encoder_derecho.getValue()
        val_gps = gps.getValues()
        val_imu = imu.getRollPitchYaw()
        
        if math.isnan(pos_izq) or math.isnan(pos_der) or math.isnan(val_gps[0]) or math.isnan(val_imu[2]):
            continue
            
        dtheta_i = pos_izq - pos_izq_ant
        dtheta_d = pos_der - pos_der_ant
        pos_izq_ant = pos_izq
        pos_der_ant = pos_der
        
        ds_i = r_rueda * dtheta_i
        ds_d = r_rueda * dtheta_d
        ds = (ds_i + ds_d) / 2.0
        dphi = (ds_d - ds_i) / L_eje
        
        x_odom += ds * math.cos(phi_odom + dphi / 2.0)
        y_odom += ds * math.sin(phi_odom + dphi / 2.0)
        phi_odom = (phi_odom + dphi + math.pi) % (2.0 * math.pi) - math.pi
        
        longitud_tray_odom += math.hypot(x_odom - pos_odom_ant[0], y_odom - pos_odom_ant[1])
        pos_odom_ant = (x_odom, y_odom)
        
        x_pred = x_est + ds * math.cos(phi_est + dphi / 2.0)
        y_pred = y_est + ds * math.sin(phi_est + dphi / 2.0)
        phi_pred = (phi_est + dphi + math.pi) % (2.0 * math.pi) - math.pi
        
        P_xx += Q_xx
        P_yy += Q_yy
        P_ff += Q_ff
        
        x_gps, y_gps, phi_imu = val_gps[0], val_gps[1], val_imu[2]
        
        K_x = P_xx / (P_xx + R_xx)
        K_y = P_yy / (P_yy + R_yy)
        K_f = P_ff / (P_ff + R_ff)
        
        x_est = x_pred + K_x * (x_gps - x_pred)
        y_est = y_pred + K_y * (y_gps - y_pred)
        
        err_f = (phi_imu - phi_pred + math.pi) % (2.0 * math.pi) - math.pi
        phi_est = (phi_pred + K_f * err_f + math.pi) % (2.0 * math.pi) - math.pi
        
        P_xx *= (1.0 - K_x)
        P_yy *= (1.0 - K_y)
        P_ff *= (1.0 - K_f)
        
        longitud_tray_real += math.hypot(x_gps - pos_real_ant[0], y_gps - pos_real_ant[1])
        pos_real_ant = (x_gps, y_gps)
        
        val_max_sensor = max(sensor.getValue() for sensor in sensores_distancia)
        if val_max_sensor > 800:
            if not en_estado_colision:
                conteo_colisiones += 1
                en_estado_colision = True
                print(f"¡Colisión detectada! Sensor Máx: {val_max_sensor:.1f}")
        else:
            en_estado_colision = False
            
        if int(tiempo_actual / dt) % 10 == 0:
            tray_real.append((x_gps, y_gps))
            tray_odom.append((x_odom, y_odom))
            tray_kalman.append((x_est, y_est))
            
        if indice_punto < total_puntos:
            meta_x, meta_y = puntos_ruta[indice_punto]
            dist_a_punto = math.hypot(meta_x - x_est, meta_y - y_est)
            
            if dist_a_punto < 0.06:
                indice_punto += 1
                print(f"Llegó al punto {indice_punto}/{total_puntos} en la posición ({x_est:.3f}, {y_est:.3f})")
                continue
                
            angulo_meta = math.atan2(meta_y - y_est, meta_x - x_est)
            error_angulo = angulo_meta - phi_est
            error_angulo = (error_angulo + math.pi) % (2.0 * math.pi) - math.pi
            
            if tiempo_recuperacion_restante > 0.0:
                tiempo_recuperacion_restante -= dt
                v = v_recuperacion
                omega = omega_recuperacion
            else:
                def obtener_valor_limpio(s):
                    return max(0.0, s.getValue() - 75.0)
                    
                c0 = obtener_valor_limpio(sensores_distancia[0])
                c7 = obtener_valor_limpio(sensores_distancia[7])
                c1 = obtener_valor_limpio(sensores_distancia[1])
                c6 = obtener_valor_limpio(sensores_distancia[6])
                c2 = obtener_valor_limpio(sensores_distancia[2])
                c5 = obtener_valor_limpio(sensores_distancia[5])
                
                suma_derecha = c0 * 3.0 + c1 * 2.0 + c2 * 1.5
                suma_izquierda = c7 * 3.0 + c6 * 2.0 + c5 * 1.5
                
                val_max_lateral = max(c0, c1, c2, c5, c6, c7)
                
                if val_max_lateral > 45.0:
                    dif_evitacion = suma_izquierda - suma_derecha
                    omega = dif_evitacion * 0.008
                    omega = max(-2.5, min(2.5, omega))
                    
                    factor_velocidad = max(0.2, 1.0 - val_max_lateral / 300.0)
                    v = V_max * factor_velocidad
                    
                    if abs(omega) < 0.2 and (c0 > 150.0 or c7 > 150.0):
                        omega = -2.0
                        v = 0.02
                else:
                    Kp_omega = 4.0
                    if abs(error_angulo) > 0.4:
                        v = 0.0
                        omega = Kp_omega * error_angulo
                    else:
                        v = V_max * (1.0 - abs(error_angulo) / 0.4)
                        omega = Kp_omega * error_angulo
                        
                if sensores_distancia[0].getValue() > 900.0 and sensores_distancia[7].getValue() > 900.0:
                    v = 0.0
                    omega = 0.0
                    if int(tiempo_actual / dt) % 20 == 0:
                        print("¡PARADA DE EMERGENCIA: Ruta completamente bloqueada al frente!")
                        
                temporizador_atascamiento += dt
                if temporizador_atascamiento > 1.0:
                    dist_movida = math.hypot(x_gps - ult_x_atascamiento, y_gps - ult_y_atascamiento)
                    if abs(v) > 0.02 and dist_movida < 0.015:
                        print(f"¡Atascamiento detectado! Se movió solo {dist_movida:.3f}m in 1s. Recuperando...")
                        tiempo_recuperacion_restante = 1.5
                        v_recuperacion = -0.04
                        if suma_derecha > suma_izquierda:
                            omega_recuperacion = 1.5
                        else:
                            omega_recuperacion = -1.5
                    ult_x_atascamiento = x_gps
                    ult_y_atascamiento = y_gps
                    temporizador_atascamiento = 0.0
                    
            omega = max(-3.0, min(3.0, omega))
            v_i = v - (omega * L_eje / 2.0)
            v_d = v + (omega * L_eje / 2.0)
            
            w_i = v_i / r_rueda
            w_d = v_d / r_rueda
            w_i = max(-6.28, min(6.28, w_i))
            w_d = max(-6.28, min(6.28, w_d))
            
            motor_izquierdo.setVelocity(w_i)
            motor_derecho.setVelocity(w_d)
        else:
            motor_izquierdo.setVelocity(0.0)
            motor_derecho.setVelocity(0.0)
            print("==========================================")
            print("¡META ALCANZADA CON ÉXITO!")
            print("==========================================")
            
            tiempo_total = tiempo_actual - tiempo_inicio
            error_pos_final = math.hypot(x_gps - x_odom, y_gps - y_odom)
            error_ang_final = abs((phi_imu - phi_odom + math.pi) % (2.0 * math.pi) - math.pi)
            
            print(f"Tiempo empleado: {tiempo_total:.2f} segundos")
            print(f"Longitud de ruta planificada: {longitud_planificada:.3f} metros")
            print(f"Longitud de ruta real: {longitud_tray_real:.3f} metros")
            print(f"Longitud de ruta por odometría: {longitud_tray_odom:.3f} metros")
            print(f"Error de posición (GPS vs Odometría): {error_pos_final:.3f} metros")
            print(f"Error de ángulo (IMU vs Odometría): {error_ang_final:.3f} radianes ({math.degrees(error_ang_final):.1f} grados)")
            print(f"Número de colisiones: {conteo_colisiones}")
            print("==========================================")
            
            ruta_bitacora = os.path.join(dir_script, "trayectorias.csv")
            try:
                with open(ruta_bitacora, 'w', encoding='utf-8') as f:
                    f.write("real_x,real_y,odom_x,odom_y,kalman_x,kalman_y\n")
                    for k in range(min(len(tray_real), len(tray_odom), len(tray_kalman))):
                        rx, ry = tray_real[k]
                        ox, oy = tray_odom[k]
                        kx, ky = tray_kalman[k]
                        f.write(f"{rx:.4f},{ry:.4f},{ox:.4f},{oy:.4f},{kx:.4f},{ky:.4f}\n")
                print(f"Bitácora de trayectoria guardada con éxito en: {ruta_bitacora}")
            except Exception as e:
                print(f"Error al escribir la bitácora de trayectoria: {e}")
            break

if __name__ == '__main__':
    main()
