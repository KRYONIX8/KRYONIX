#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np
import RPi.GPIO as GPIO

# -------------------------------------------------
# CONFIGURACIÓN GENERAL
# -------------------------------------------------

# True  -> trayectoria HORARIA (gira a DERECHA)
# False -> trayectoria ANTIHORARIA (gira a IZQUIERDA)
CLOCKWISE = True

# Vueltas a dar y tiempo aproximado por vuelta
NUM_LAPS = 3
LAP_TIME_SEC = 25.0  # AJUSTAR EN PISTA REAL

# -------------------------------------------------
# PINES (según tu conexión)
# -------------------------------------------------

# Motor DC (DRV8870)
PIN_MOTOR_N1 = 20       # IN1
PIN_MOTOR_N2 = 21       # IN2
PIN_MOTOR_EN = 25       # Enable (PWM)

# Servo MG996R
PIN_SERVO = 18          # GPIO18, PWM HW

# HC-SR04 frontal
PIN_TRIG_FRONT = 23
PIN_ECHO_FRONT = 24

# HC-SR04 trasero
PIN_TRIG_BACK = 5
PIN_ECHO_BACK = 6

# VL53L0X (XSHUT)
PIN_XSHUT_RIGHT = 17    # derecha
PIN_XSHUT_LEFT = 27     # izquierda

# -------------------------------------------------
# PARÁMETROS DEL CONTROL
# -------------------------------------------------

# Servo (duty cycle aproximado, AJUSTAR)
SERVO_CENTER = 7.5
SERVO_RIGHT  = 6.0
SERVO_LEFT   = 9.0

# Motor (PWM 0–100)
MOTOR_SPEED_MAX  = 90
MOTOR_SPEED_SLOW = 60

# Distancias de seguridad (cm)
FRONT_STOP_DIST = 25.0
BACK_SAFE_DIST  = 15.0

# Visión
WHITE_GO_THRESHOLD   = 0.65
WHITE_LOST_THRESHOLD = 0.45
MAX_TURN_TIME        = 3.0

# -------------------------------------------------
# VARIABLES DE ESTADO (para GUI)
# -------------------------------------------------
current_motor_speed = 0      # -100..100 (signo = sentido)
current_servo_duty = SERVO_CENTER
current_routine = "Iniciando..."
last_distances_mm = {
    "front": None,
    "back": None,
    "right": None,
    "left": None,
}

# -------------------------------------------------
# INICIALIZACIÓN GPIO
# -------------------------------------------------

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor
GPIO.setup(PIN_MOTOR_N1, GPIO.OUT)
GPIO.setup(PIN_MOTOR_N2, GPIO.OUT)
GPIO.setup(PIN_MOTOR_EN, GPIO.OUT)
pwm_motor = GPIO.PWM(PIN_MOTOR_EN, 20000)  # 20 kHz
pwm_motor.start(0)

# Servo
GPIO.setup(PIN_SERVO, GPIO.OUT)
pwm_servo = GPIO.PWM(PIN_SERVO, 50)        # 50 Hz
pwm_servo.start(SERVO_CENTER)

# Ultrasónicos
GPIO.setup(PIN_TRIG_FRONT, GPIO.OUT)
GPIO.setup(PIN_ECHO_FRONT, GPIO.IN)
GPIO.setup(PIN_TRIG_BACK, GPIO.OUT)
GPIO.setup(PIN_ECHO_BACK, GPIO.IN)

# VL53 XSHUT
GPIO.setup(PIN_XSHUT_RIGHT, GPIO.OUT)
GPIO.setup(PIN_XSHUT_LEFT, GPIO.OUT)

# -------------------------------------------------
# INICIALIZACIÓN VL53L0X
# -------------------------------------------------
try:
    from VL53L0X import VL53L0X

    GPIO.output(PIN_XSHUT_RIGHT, GPIO.LOW)
    GPIO.output(PIN_XSHUT_LEFT, GPIO.LOW)
    time.sleep(0.1)

    # Derecha: encender y cambiar dirección
    GPIO.output(PIN_XSHUT_RIGHT, GPIO.HIGH)
    time.sleep(0.1)
    tof_right = VL53L0X(address=0x29)
    tof_right.change_address(0x2A)
    time.sleep(0.1)

    # Izquierda: encender con dirección por defecto
    GPIO.output(PIN_XSHUT_LEFT, GPIO.HIGH)
    time.sleep(0.1)
    tof_left = VL53L0X(address=0x29)

    tof_right.start_ranging(VL53L0X.BEST_ACCURACY_MODE)
    tof_left.start_ranging(VL53L0X.BEST_ACCURACY_MODE)

    VL53_AVAILABLE = True
except Exception as e:
    print("No se pudo inicializar VL53L0X:", e)
    tof_right = None
    tof_left = None
    VL53_AVAILABLE = False

# -------------------------------------------------
# FUNCIONES BAJO NIVEL (motor / servo)
# -------------------------------------------------

def set_servo_center():
    global current_servo_duty
    current_servo_duty = SERVO_CENTER
    pwm_servo.ChangeDutyCycle(current_servo_duty)

def set_servo_turn_right():
    global current_servo_duty
    duty = SERVO_RIGHT if CLOCKWISE else SERVO_LEFT
    current_servo_duty = duty
    pwm_servo.ChangeDutyCycle(duty)

def set_servo_turn_left():
    global current_servo_duty
    duty = SERVO_LEFT if CLOCKWISE else SERVO_RIGHT
    current_servo_duty = duty
    pwm_servo.ChangeDutyCycle(duty)

def motor_stop():
    global current_motor_speed
    GPIO.output(PIN_MOTOR_N1, GPIO.LOW)
    GPIO.output(PIN_MOTOR_N2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(0)
    current_motor_speed = 0

def motor_forward(speed):
    global current_motor_speed
    GPIO.output(PIN_MOTOR_N1, GPIO.HIGH)
    GPIO.output(PIN_MOTOR_N2, GPIO.LOW)
    pwm_motor.ChangeDutyCycle(speed)
    current_motor_speed = abs(speed)

def motor_backward(speed):
    global current_motor_speed
    GPIO.output(PIN_MOTOR_N1, GPIO.LOW)
    GPIO.output(PIN_MOTOR_N2, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed)
    current_motor_speed = -abs(speed)

# -------------------------------------------------
# ULTRASÓNICOS / VL53
# -------------------------------------------------

def read_ultrasonic(trig_pin, echo_pin, name_key, timeout=0.03):
    GPIO.output(trig_pin, GPIO.LOW)
    time.sleep(0.000002)
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    start_time = time.time()
    while GPIO.input(echo_pin) == 0:
        if time.time() - start_time > timeout:
            last_distances_mm[name_key] = None
            return None
    pulse_start = time.time()

    while GPIO.input(echo_pin) == 1:
        if time.time() - pulse_start > timeout:
            last_distances_mm[name_key] = None
            return None
    pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance_cm = (pulse_duration * 34300) / 2.0
    distance_mm = distance_cm * 10.0
    last_distances_mm[name_key] = distance_mm
    return distance_cm

def read_front_distance():
    return read_ultrasonic(PIN_TRIG_FRONT, PIN_ECHO_FRONT, "front")

def read_back_distance():
    return read_ultrasonic(PIN_TRIG_BACK, PIN_ECHO_BACK, "back")

def read_side_distances():
    if not VL53_AVAILABLE:
        last_distances_mm["right"] = None
        last_distances_mm["left"] = None
        return None, None
    try:
        d_right_mm = tof_right.get_distance()
        d_left_mm  = tof_left.get_distance()
        last_distances_mm["right"] = d_right_mm
        last_distances_mm["left"]  = d_left_mm
        return d_right_mm / 10.0, d_left_mm / 10.0  # cm
    except Exception:
        last_distances_mm["right"] = None
        last_distances_mm["left"] = None
        return None, None

# -------------------------------------------------
# VISIÓN
# -------------------------------------------------

def process_frame(frame):
    """
    Devuelve:
    - white_ratio: proporción de blancos en la ROI
    - balance: diferencia de blanco izquierda-derecha
    """
    frame_resized = cv2.resize(frame, (320, 240))
    roi = frame_resized[120:240, :]   # mitad inferior

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 160, 255, cv2.THRESH_BINARY)

    white_pixels = np.count_nonzero(thresh == 255)
    total_pixels = thresh.size
    white_ratio = white_pixels / float(total_pixels)

    left_roi = thresh[:, :160]
    right_roi = thresh[:, 160:]
    white_left = np.count_nonzero(left_roi == 255)
    white_right = np.count_nonzero(right_roi == 255)
    if white_left + white_right == 0:
        balance = 0.0
    else:
        balance = (white_left - white_right) / float(white_left + white_right)

    return white_ratio, balance

# -------------------------------------------------
# GUI / HUD SOBRE EL FRAME
# -------------------------------------------------

def duty_to_angle(duty):
    """
    Conversión aproximada de duty 5-10% a 0-180°
    (ajustar si tu servo usa otro rango).
    """
    return (duty - 5.0) * (180.0 / 5.0)

def draw_hud(frame):
    """
    Dibuja cámara + texto de estado sobre el frame.
    Usa variables globales de estado.
    """
    global current_motor_speed, current_servo_duty, current_routine, last_distances_mm

    # Redimensionar para que quepa todo
    frame = cv2.resize(frame, (640, 480))

    # Panel semitransparente
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (640, 140), (0, 0, 0), -1)
    alpha = 0.6
    frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

    # Texto principal
    font = cv2.FONT_HERSHEY_SIMPLEX
    color_text = (255, 255, 255)
    y = 25

    # Rutina actual
    cv2.putText(frame, f"Rutina: {current_routine}", (10, y),
                font, 0.6, color_text, 1, cv2.LINE_AA)
    y += 25

    # Velocidad del motor
    if current_motor_speed > 0:
        sentido = "Avance"
    elif current_motor_speed < 0:
        sentido = "Reversa"
    else:
        sentido = "Detenido"

    cv2.putText(frame,
                f"Motor: {abs(current_motor_speed):3.0f}% ({sentido})",
                (10, y), font, 0.6, color_text, 1, cv2.LINE_AA)
    y += 25

    # Ángulo del servo
    angle = duty_to_angle(current_servo_duty)
    cv2.putText(frame,
                f"Servo: {angle:5.1f} grados",
                (10, y), font, 0.6, color_text, 1, cv2.LINE_AA)
    y += 30

    # Tabla de distancias
    cv2.putText(frame, "Distancias (mm):", (10, y),
                font, 0.6, color_text, 1, cv2.LINE_AA)
    y += 22

    def fmt(name):
        val = last_distances_mm[name]
        return "---" if val is None else f"{val:7.1f}"

    cv2.putText(frame, f"Front: {fmt('front')}", (20, y),
                font, 0.5, color_text, 1, cv2.LINE_AA)
    y += 20
    cv2.putText(frame, f"Back : {fmt('back')}", (20, y),
                font, 0.5, color_text, 1, cv2.LINE_AA)
    y += 20
    cv2.putText(frame, f"Right: {fmt('right')}", (20, y),
                font, 0.5, color_text, 1, cv2.LINE_AA)
    y += 20
    cv2.putText(frame, f"Left : {fmt('left')}", (20, y),
                font, 0.5, color_text, 1, cv2.LINE_AA)

    # Instrucción de salida
    cv2.putText(frame, "Presiona 'q' para salir",
                (360, 470), font, 0.6, (0, 255, 255), 1, cv2.LINE_AA)

    return frame

# -------------------------------------------------
# COMPORTAMIENTO ALTO NIVEL
# -------------------------------------------------

def adjust_steering(balance):
    """
    Ajuste simple según balance de blanco izquierda/derecha.
    """
    if abs(balance) < 0.1:
        set_servo_center()
        return
    if balance > 0:
        set_servo_turn_left()
    else:
        set_servo_turn_right()

def avoid_obstacle(front_dist, back_dist, cap):
    """
    - Detenerse
    - Retroceder (si hay espacio)
    - Girar en el sentido principal hasta volver a ver blanco
    Mostrando la GUI en todo el proceso.
    """
    global current_routine

    # Detener
    current_routine = "Detenido por obstaculo"
    motor_stop()
    time.sleep(0.2)

    # Retroceder
    if back_dist is None or back_dist > BACK_SAFE_DIST:
        current_routine = "Evitando: retrocediendo"
        set_servo_center()
        t0 = time.time()
        motor_backward(MOTOR_SPEED_SLOW)
        while time.time() - t0 < 0.5:
            ret, frame = cap.read()
            if not ret:
                continue
            frame_hud = draw_hud(frame)
            cv2.imshow("Carrito - Vision y Telemetria", frame_hud)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                motor_stop()
                return True  # salir de todo
        motor_stop()
        time.sleep(0.1)

    # Girar buscando blanco
    current_routine = "Evitando: girando/buscando blanco"
    if CLOCKWISE:
        set_servo_turn_right()
    else:
        set_servo_turn_left()
    motor_forward(MOTOR_SPEED_SLOW)

    turn_start = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        white_ratio, _ = process_frame(frame)
        frame_hud = draw_hud(frame)
        cv2.imshow("Carrito - Vision y Telemetria", frame_hud)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            motor_stop()
            return True  # terminar programa

        if white_ratio > WHITE_GO_THRESHOLD:
            break
        if time.time() - turn_start > MAX_TURN_TIME:
            break

    motor_stop()
    set_servo_center()
    time.sleep(0.1)
    return False

# -------------------------------------------------
# PROGRAMA PRINCIPAL
# -------------------------------------------------

def main():
    global current_routine

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("No se pudo abrir la camara")
        return

    print("Iniciando recorrido...")
    current_routine = "Iniciando recorrido"

    start_time = time.time()
    end_time = start_time + NUM_LAPS * LAP_TIME_SEC

    try:
        while time.time() < end_time:
            ret, frame = cap.read()
            if not ret:
                print("Frame no valido")
                continue

            white_ratio, balance = process_frame(frame)
            front_dist = read_front_distance()
            back_dist  = read_back_distance()
            side_right, side_left = read_side_distances()

            # 1) Obstáculo frontal
            if front_dist is not None and front_dist < FRONT_STOP_DIST:
                salir = avoid_obstacle(front_dist, back_dist, cap)
                if salir:
                    break
                continue

            # 2) Piso muy negro (perdimos la ruta)
            if white_ratio < WHITE_LOST_THRESHOLD:
                current_routine = "Buscando piso blanco"
                motor_stop()
                salir = avoid_obstacle(front_dist, back_dist, cap)
                if salir:
                    break
                continue

            # 3) Piso mayormente blanco
            if white_ratio > WHITE_GO_THRESHOLD:
                current_routine = "Avanzando rapido"
                adjust_steering(balance)
                motor_forward(MOTOR_SPEED_MAX)
            else:
                current_routine = "Avanzando lento"
                adjust_steering(balance)
                motor_forward(MOTOR_SPEED_SLOW)

            # Dibuja interfaz
            frame_hud = draw_hud(frame)
            cv2.imshow("Carrito - Vision y Telemetria", frame_hud)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                current_routine = "Detenido por usuario"
                break

            time.sleep(0.03)

        current_routine = "Vueltas completadas"
        motor_stop()
        set_servo_center()
        # mostrar ultimo frame un momento
        ret, frame = cap.read()
        if ret:
            frame_hud = draw_hud(frame)
            cv2.imshow("Carrito - Vision y Telemetria", frame_hud)
            cv2.waitKey(2000)

    except KeyboardInterrupt:
        print("Interrupcion con Ctrl+C")

    finally:
        motor_stop()
        set_servo_center()
        cap.release()
        pwm_servo.stop()
        pwm_motor.stop()
        GPIO.cleanup()
        cv2.destroyAllWindows()
        if VL53_AVAILABLE:
            try:
                tof_right.stop_ranging()
                tof_left.stop_ranging()
            except Exception:
                pass


if __name__ == "__main__":
    main()