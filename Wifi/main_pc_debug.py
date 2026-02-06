#
# Este código necesita también del código: vision.py
#
# La función es recibir el video de la raspberry y procesarlo con vision, escribir el color que se encontró en la terminal al igual que su centroide y área.
#
# Aquí se incluyen funciones para debuggear, transmite a la página y se abre el cuadro de video con bounding boxes y centroides.

from vision import detect_colors, crosslines
import cv2 as cv
import requests
import time

PI_IP = "172.32.236.53"

VIDEO_URL  = f"http://{PI_IP}:5000/video_feed"
CMD_URL    = f"http://{PI_IP}:5000/command"
VISION_URL = f"http://{PI_IP}:5000/vision_data"

def send_rpms(left_rpm, right_rpm, dir_left, dir_right):   
    speed = {
        "left_rpm": f"S{left_rpm}",
        "right_rpm": f"S{right_rpm}",
        "left_dir": f"D{dir_left}",
        "right_dir": f"D{dir_right}"
    }

    try:
        requests.post(
            CMD_URL,
            json=speed,
            timeout=0.5 # tiempo máximo que tu programa esperará una respuesta del servidor antes de darse por vencido
        )
    except Exception as e:
        print("Error enviando rpms:", e)


def send_vision_data(colors, centroids, areas):
    payload = {
        "colors": colors,
        "centroids": centroids,
        "areas": areas,
        "time": time.time()
    }

    try:
        requests.post(
            VISION_URL,
            json=payload,
            timeout=0.5
        )
    except Exception as e:
        print("Error enviando vision_data:", e)


def calc_turn_x(centroid, frame_width, deadband_px=10):
    cx = centroid[1]
    center_x = frame_width / 2
    error_px = cx - center_x

    if abs(error_px) < deadband_px:
        return 0.0

    error = error_px / (frame_width / 2)   # [-1..1]
    Kp = 5                                
    turn = Kp * error

    turn = max(-1.0, min(1.0, turn))      
    return turn


def calc_base_y(centroid, frame_height, y_trigger_ratio=0.40):
    cy = centroid[2]
    y_trigger = frame_height * y_trigger_ratio

    # Si el objeto aún está “arriba” (lejos), avanza
    if cy < y_trigger:
        return 1.0   # avanzar (normalizado)
    else:
        return 0.0   # ya llegó -> no avanzar


def pick_target(centroids, areas, area_min=1500):
    if not centroids or not areas:
        return None  # no hay nada

    # Filtra por área mínima
    candidates = [(c, a) for c, a in zip(centroids, areas) if a >= area_min]
    if not candidates:
        return None

    # Escoge el de mayor área
    centroid, area = max(candidates, key=lambda x: x[1])
    return centroid, area

# Limita RPM: 0 (paro) o >= min_rpm
def clamp_rpm(rpm, min_rpm=20):
    if rpm < min_rpm:
        return 0
    return int(rpm)

def main():
    cap = cv.VideoCapture(VIDEO_URL)

    if not cap.isOpened():
        print("No se pudo abrir el stream")
        return

    print("Stream conectado")

    # --- Parámetros de estabilidad ---
    N_ENTER = 5     # frames consecutivos para entrar a tracking (rápido)
    N_EXIT  = 10    # frames consecutivos sin target para salir (más lento)
    AREA_MIN = 500 # ajusta según tu escena (filtro anti-ruido)

    seen_count = 0
    lost_count = 0
    tracking = False


    CRUISE_RPM = 30    # velocidad default cuando no hay tracking
    APPROACH_RPM = 25  # velocidad al acercarse
    left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM  # velocidad default
    dir_left, dir_right = 1, 1     # dirección default (1=forward, 0=backward)

    vision_send_counter = 0
    VISION_SEND_EVERY = 5  # Envía cada 10 frames

    command_send_counter = 0
    COMMAND_SEND_EVERY = 2 # Envía cada 5 frames

    mode_rotate = False  # Estado: ¿estamos en modo rotación?
    Y_TRIGGER = 0.40
    Y_HYST = 0.35  # histéresis: 5% de diferencia

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame no recibido")
            break

        # Detección
        colors, centroids, areas = detect_colors(frame, draw=True)

        # Decide si hay un target "confiable"
        target = pick_target(centroids, areas, area_min=AREA_MIN)
        detected = target is not None

        if detected:
            centroid = target[0]

        # --- Lógica de contadores ---
        if detected:
            seen_count += 1
            lost_count = 0
        else:
            lost_count += 1
            seen_count = 0

        # --- Transiciones de modo (tracking ON/OFF) ---
        if (not tracking) and (seen_count >= N_ENTER):
            tracking = True
            print("Tracking ACTIVADO (detección estable)\n")

        if tracking and (lost_count >= N_EXIT):
            tracking = False
            print("Tracking DESACTIVADO (detección perdida)")
            # Aquí vuelves al modo default

        if not tracking:
            left_rpm, right_rpm = CRUISE_RPM, CRUISE_RPM
            dir_left, dir_right = 1, 1
            
        # Dentro del tracking:
        if tracking and detected:
            centroid, area = target
            h, w = frame.shape[0], frame.shape[1]
            
            cy = centroid[2]
            turn = calc_turn_x(centroid, w)
            
            # Histéresis para evitar oscilaciones
            if not mode_rotate and cy >= h * Y_TRIGGER:
                mode_rotate = True  # Entra a modo rotación
            elif mode_rotate and cy < h * Y_HYST:
                mode_rotate = False  # Sale de modo rotación
            
            if mode_rotate:
                # Modo rotación pura
                rot = APPROACH_RPM * abs(turn)
                
                # Solo gira si el error es significativo
                if abs(turn) < 0.1:  # deadband de rotación
                    left_rpm, right_rpm = 0, 0
                    dir_left, dir_right = 1, 1
                elif turn > 0:
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 0, 1
                else:
                    left_rpm, right_rpm = rot, rot
                    dir_left, dir_right = 1, 0
            else:
                # Modo avance
                left_rpm, right_rpm = APPROACH_RPM, APPROACH_RPM
                dir_left, dir_right = 1, 1


        # --- Envío de comandos ---
        left_rpm_clamped = clamp_rpm(left_rpm, min_rpm=20)
        right_rpm_clamped = clamp_rpm(right_rpm, min_rpm=20)
            
        # --- Cálculo de comandos ---
        command_send_counter += 1
        if command_send_counter >= COMMAND_SEND_EVERY:
            send_rpms(left_rpm_clamped, right_rpm_clamped, dir_left, dir_right)
            command_send_counter = 0

        vision_send_counter += 1
        if vision_send_counter >= VISION_SEND_EVERY:
            send_vision_data(colors, centroids, areas)
            vision_send_counter = 0

        # Visualización
        video = crosslines(frame.copy())
        cv.imshow("VISION ROVER (PC DEBUG)", video)

        if cv.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
