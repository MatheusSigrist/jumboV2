import sensor
import time
import math
import struct
from machine import UART

# ===== COMUNICAÇÃO SERIAL COM OLHO =====
uart_olho = UART(1, 19200, timeout_char=200)

# ===== PROTOCOLO SERIAL =====
BYTE_INICIA = 0xAA
BYTE_PARA = 0x55
ID_PLACA_CAMERA = 0x03

# =========================================================
# CONFIGURAÇÕES GERAIS
# =========================================================

R = 105
cx = 172
cy = 110
R2 = R * R

DEBUG = True
center = [172, 110]

# =========================================================
# ZONA DA BOLA
# =========================================================
BALL_ZONE_INNER = 25
BALL_ZONE_OUTER = 90

BALL_ZONE_INNER2 = BALL_ZONE_INNER * BALL_ZONE_INNER
BALL_ZONE_OUTER2 = BALL_ZONE_OUTER * BALL_ZONE_OUTER

# =========================================================
# ZONA DOS GOLS
# =========================================================
GOAL_ZONE_OUTER = 120
GOAL_ZONE_OUTER2 = GOAL_ZONE_OUTER * GOAL_ZONE_OUTER

# =========================================================
# FUNÇÕES AUXILIARES
# =========================================================

def calc_goal_angle_and_distance(x, y, center_x, center_y):
    """
    GOLS:
    Mantém a lógica antiga:
    - ângulo em faixa negativa/positiva
    """
    dx = x - center_x
    dy = y - center_y

    angle = -((((math.atan2(dx, dy) * 180) / math.pi) + 360) % 360 - 180)
    distance = math.sqrt(dx**2 + dy**2)

    return int(angle), int(distance)


def calc_ball_angle_and_distance(x, y, center_x, center_y):
    """
    BOLA:
    - 0 a 359 graus
    - corrigida para compensar a imagem espelhada
    """
    dx = x - center_x
    dy = y - center_y

    angle = (360 - ((math.degrees(math.atan2(dx, dy)) + 360) % 360) + 180) % 360

    # corrige o espelhamento lateral da imagem
    angle = (360 - angle) % 360

    distance = math.sqrt(dx**2 + dy**2)

    return int(angle), int(distance)


def is_in_ball_zone(x, y):
    dx = x - center[0]
    dy = y - center[1]
    d2 = dx * dx + dy * dy
    return (d2 >= BALL_ZONE_INNER2) and (d2 <= BALL_ZONE_OUTER2)


def is_in_goal_zone(x, y):
    dx = x - center[0]
    dy = y - center[1]
    d2 = dx * dx + dy * dy
    return (d2 <= GOAL_ZONE_OUTER2)


def find_best_ball_blob_in_zone(img, threshold, pixels_threshold, area_threshold,
                                merge, margin, min_pixels=None, max_pixels=None):
    best_blob = None
    best_perimeter = 0

    for blob in img.find_blobs([threshold],
                               pixels_threshold=pixels_threshold,
                               area_threshold=area_threshold,
                               merge=merge,
                               margin=margin):

        px = blob.pixels()

        if min_pixels is not None and px < min_pixels:
            continue
        if max_pixels is not None and px > max_pixels:
            continue

        if not is_in_ball_zone(blob.cx(), blob.cy()):
            continue

        if blob.perimeter() > best_perimeter:
            best_perimeter = blob.perimeter()
            best_blob = blob

    return best_blob


def find_best_goal_blob_in_zone(img, threshold, pixels_threshold, area_threshold,
                                merge, margin):
    best_blob = None
    best_pixels = 0

    for blob in img.find_blobs([threshold],
                               pixels_threshold=pixels_threshold,
                               area_threshold=area_threshold,
                               merge=merge,
                               margin=margin):

        px = blob.pixels()

        if not is_in_goal_zone(blob.cx(), blob.cy()):
            continue

        if px > best_pixels:
            best_pixels = px
            best_blob = blob

    return best_blob


def draw_blob_info(img, blob, color_rgb, label, angle, distance, extra_text=""):
    img.draw_rectangle(blob.rect(), color=color_rgb, thickness=2)
    img.draw_cross(blob.cx(), blob.cy(), color=color_rgb, size=10, thickness=2)

    text1 = "{} A:{} D:{}".format(label, angle, distance)
    img.draw_string(blob.x(), max(blob.y() - 20, 0), text1, color=color_rgb, scale=1)

    if extra_text:
        img.draw_string(blob.x(), max(blob.y() - 8, 0), extra_text, color=color_rgb, scale=1)


def enviar_dados_visao(ball_angle, ball_dist, blue_angle, blue_dist, yellow_angle, yellow_dist):
    """
    Envia dados de BOLA + 2 GOLS via serial para placa OLHO
    Estrutura: [BALL_A(h)][BALL_D(H)][BLUE_A(h)][BLUE_D(H)][YELLOW_A(h)][YELLOW_D(H)]
    h = int16 (ângulo), H = uint16 (distância)
    Total: 2+2+2+2+2+2 = 12 bytes de dados
    """
    # Normaliza ângulos para range -180 a 180 (compatível com int16)
    ball_a_raw = int(ball_angle) if ball_angle is not None else -999
    ball_d_raw = int(ball_dist) if ball_dist is not None else 0
    blue_a_raw = int(blue_angle) if blue_angle is not None else -999
    blue_d_raw = int(blue_dist) if blue_dist is not None else 0
    yellow_a_raw = int(yellow_angle) if yellow_angle is not None else -999
    yellow_d_raw = int(yellow_dist) if yellow_dist is not None else 0

    # Limita valores para range válido de int16/uint16
    ball_a_raw = max(-32768, min(32767, ball_a_raw))
    ball_d_raw = max(0, min(65535, ball_d_raw))
    blue_a_raw = max(-32768, min(32767, blue_a_raw))
    blue_d_raw = max(0, min(65535, blue_d_raw))
    yellow_a_raw = max(-32768, min(32767, yellow_a_raw))
    yellow_d_raw = max(0, min(65535, yellow_d_raw))

    # Empacota dados: > = big-endian, h = int16, H = uint16
    msg = struct.pack(">hHhHhH", 
                      ball_a_raw, ball_d_raw,
                      blue_a_raw, blue_d_raw,
                      yellow_a_raw, yellow_d_raw)

    # Framing: BYTE_INICIA + ID + dados(12) + BYTE_PARA
    buffer = bytearray(1 + 1 + 12 + 1)
    buffer[0] = BYTE_INICIA
    buffer[1] = ID_PLACA_CAMERA
    buffer[2:14] = msg
    buffer[14] = BYTE_PARA

    uart_olho.write(buffer)


# =========================================================
# THRESHOLDS
# =========================================================

thresholdb = [15, 20, -11, 15, -20, 0]   # azul
thresholdy = [47, 50, 8, 20, 45, 5]      # amarelo
thresholdo = [31, 48, -9, 27, 17, 35]    # laranja

# =========================================================
# CÂMERA
# =========================================================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=1000)

sensor.set_auto_whitebal(False, rgb_gain_db=(62, 60, 64))
sensor.set_auto_exposure(False, exposure_us=25000)
sensor.set_auto_gain(False, gain_db=10)

print("RGB gain:", sensor.get_rgb_gain_db())
print("Exposure:", sensor.get_exposure_us())
print("Gain:", sensor.get_gain_db())

sensor.skip_frames(time=1000)

clock = time.clock()
print("Iniciando detecção...")

# =========================================================
# LOOP PRINCIPAL
# =========================================================

while True:
    clock.tick()
    img = sensor.snapshot()

    # Máscara quadrada externa
    left = cx - R
    right = cx + R
    top = cy - R
    bottom = cy + R

    img.draw_rectangle(0, 0, img.width(), top, color=(0, 0, 0), fill=True)
    img.draw_rectangle(0, bottom, img.width(), img.height() - bottom, color=(0, 0, 0), fill=True)
    img.draw_rectangle(0, top, left, bottom - top, color=(0, 0, 0), fill=True)
    img.draw_rectangle(right, top, img.width() - right, bottom - top, color=(0, 0, 0), fill=True)

    # DEBUG DAS ZONAS
    if DEBUG:
        img.draw_cross(center[0], center[1], color=(255, 255, 255), size=12)
        img.draw_circle(center[0], center[1], BALL_ZONE_INNER, color=(80, 80, 80))
        img.draw_circle(center[0], center[1], BALL_ZONE_OUTER, color=(255, 140, 0))
        img.draw_circle(center[0], center[1], GOAL_ZONE_OUTER, color=(0, 255, 0))

    # BOLA
    orange_blob = find_best_ball_blob_in_zone(
        img,
        threshold=thresholdo,
        pixels_threshold=10,
        area_threshold=10,
        merge=True,
        margin=3,
        min_pixels=5,
        max_pixels=100
    )

    orange_found = 0
    orange_angle = 0
    orange_dist = 0

    if orange_blob is not None:
        orange_found = 1
        orange_angle, orange_dist = calc_ball_angle_and_distance(
            orange_blob.cx(), orange_blob.cy(), center[0], center[1]
        )

        if DEBUG:
            draw_blob_info(img, orange_blob, (255, 140, 0), "BALL", orange_angle, orange_dist, "ZB:1")

    # GOL AZUL
    blue_blob = find_best_goal_blob_in_zone(
        img,
        threshold=thresholdb,
        pixels_threshold=150,
        area_threshold=150,
        merge=True,
        margin=10
    )

    blue_found = 0
    blue_angle = 0
    blue_dist = 0

    if blue_blob is not None:
        blue_found = 1
        blue_angle, blue_dist = calc_goal_angle_and_distance(
            blue_blob.cx(), blue_blob.cy(), center[0], center[1]
        )

        if DEBUG:
            extra = "ZG:1 PX:{} AR:{}".format(blue_blob.pixels(), blue_blob.area())
            draw_blob_info(img, blue_blob, (0, 0, 255), "BLUE", blue_angle, blue_dist, extra)

    # GOL AMARELO
    yellow_blob = find_best_goal_blob_in_zone(
        img,
        threshold=thresholdy,
        pixels_threshold=50,
        area_threshold=1,
        merge=True,
        margin=10
    )

    yellow_found = 0
    yellow_angle = 0
    yellow_dist = 0

    if yellow_blob is not None:
        yellow_found = 1
        yellow_angle, yellow_dist = calc_goal_angle_and_distance(
            yellow_blob.cx(), yellow_blob.cy(), center[0], center[1]
        )

        if DEBUG:
            extra = "ZG:1 PX:{} AR:{}".format(yellow_blob.pixels(), yellow_blob.area())
            draw_blob_info(img, yellow_blob, (255, 255, 0), "YELL", yellow_angle, yellow_dist, extra)

    # ===== ENVIA DADOS VIA SERIAL =====
    enviar_dados_visao(orange_angle, orange_dist, blue_angle, blue_dist, yellow_angle, yellow_dist)

    # DEBUG NO TERMINAL
    print("FPS: {:.2f} | BALL:{} A:{} D:{} | BLUE:{} A:{} D:{} | YELL:{} A:{} D:{}".format(
        clock.fps(),
        orange_found, orange_angle, orange_dist,
        blue_found, blue_angle, blue_dist,
        yellow_found, yellow_angle, yellow_dist
    ))
