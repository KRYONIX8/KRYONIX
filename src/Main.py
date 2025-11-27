import cv2
import time
import threading
import numpy as np
import traceback
import json
import os
from flask import Flask, render_template_string, Response, jsonify, request
import RPi.GPIO as GPIO
import board
import busio
from adafruit_vl53l0x import VL53L0X

# ==========================================
# 1. CONFIGURACIÓN Y PINES
# ==========================================
TRIG_FRONT = 23
ECHO_FRONT = 24
XSHUT_LEFT = 5
XSHUT_RIGHT = 6
SERVO_PIN = 18
MOTOR_RPWM = 12
MOTOR_LPWM = 13
MOTOR_ENABLE = 19

# Pines del Switch de Competencia
SWITCH_PWR_PIN = 8   # Salida 3.3V
SWITCH_READ_PIN = 11 # Entrada

# Resolución optimizada para velocidad
FRAME_WIDTH = 160
FRAME_HEIGHT = 120
CONFIG_FILE = "robot_config.json"

# Configuración Por Defecto
default_config = {
    # Visión
    "black_max_v": 80,    
    "white_min_v": 160,   
    "sat_min": 60,        
    "orange_hue": 15,     
    "blue_hue": 110,      
    "hue_margin": 20,
    "roi_top": 30,    
    "roi_bottom": 100, 
    
    # Navegación
    "base_speed": 60,       
    "reverse_speed": 50,    
    "reverse_time": 0.8,    
    "min_front_dist": 30,   # Distancia choque frontal
    "slow_dist": 100,       
    "side_stuck_dist": 75,  # NUEVO: Distancia para considerar atasco lateral (mm)
    
    # Servo
    "servo_center": 70,
    "servo_min": 40,
    "servo_max": 100
}

# Estado Global
robot_state = {
    "mode": "Manual",
    "status": "Stopped",
    "switch_active": False, # Estado físico del switch
    "view_mode": "normal", 
    "lap_count": 0.0,
    "sensor_front": 0,
    "sensor_left": 0,
    "sensor_right": 0,
    "visual_wall": False,
    "is_stuck": False,
    "chosen_direction": None,
    "servo_angle": 70,
    "motor_speed": 0,
    "motor_enable_state": True,
    "error_log": "Listo",
    "config": default_config.copy() 
}

# Variables internas
lap_state = 0
last_movement_ts = time.time()
tof_stuck_counter = 0
last_tof_readings = (0, 0)
sensor_reset_timer = time.time()

lock = threading.Lock()

# ==========================================
# 2. GESTIÓN DE MEMORIA
# ==========================================
def load_config():
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, 'r') as f:
                loaded = json.load(f)
                for k, v in loaded.items():
                    if k in robot_state["config"]:
                        robot_state["config"][k] = v
        except: pass

def save_config():
    try:
        with open(CONFIG_FILE, 'w') as f:
            json.dump(robot_state["config"], f, indent=4)
    except: pass

load_config()

# ==========================================
# 3. INTERFAZ HTML
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Control Center</title>
    <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/css/bootstrap.min.css" rel="stylesheet">
    <style>
        body { background-color: #f8f9fa; color: #343a40; font-family: 'Segoe UI', sans-serif; }
        .card { background-color: #ffffff; border: 1px solid #e9ecef; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.02); margin-bottom: 20px; }
        .nav-tabs .nav-link { color: #6c757d; font-weight: 500; border: none; }
        .nav-tabs .nav-link.active { color: #0d6efd; border-bottom: 2px solid #0d6efd; background: transparent; }
        label { font-size: 0.85rem; color: #495057; font-weight: 600; text-transform: uppercase; letter-spacing: 0.5px; }
        .val-badge { background: #e9ecef; color: #212529; padding: 2px 8px; border-radius: 4px; font-size: 0.8rem; font-family: monospace; float: right; }
        .camera-feed { width: 100%; border-radius: 8px; box-shadow: 0 2px 8px rgba(0,0,0,0.1); background: #000; min-height: 240px; }
        .sensor-val { font-size: 1.5rem; font-weight: 700; color: #0d6efd; display: block; }
        .status-badge { font-size: 0.9rem; padding: 5px 10px; border-radius: 20px; }
        .bg-running { background-color: #d1e7dd; color: #0f5132; }
        .bg-stopped { background-color: #f8d7da; color: #842029; }
        .bg-switch { background-color: #fff3cd; color: #856404; border: 1px solid #ffeeba; }
    </style>
</head>
<body>
    <div class="container py-4">
        <div class="d-flex justify-content-between align-items-center mb-4">
            <h4 class="m-0">🏎️ Robot Dashboard</h4>
            <div id="switch-indicator" class="d-none status-badge bg-switch fw-bold">⚠️ SWITCH FÍSICO ACTIVO</div>
            <button class="btn btn-outline-secondary btn-sm" onclick="location.reload()">🔄 UI Reset</button>
        </div>

        <div class="row">
            <div class="col-lg-7">
                <div class="card p-3">
                    <div class="position-relative">
                        <img src="/video_feed" class="camera-feed" id="cam-feed">
                        <div id="alert-overlay" class="position-absolute top-50 start-50 translate-middle text-center d-none" style="background: rgba(255,0,0,0.8); padding: 15px; border-radius: 8px; color: white;">
                            <h3 class="m-0">🛑 OBSTÁCULO / ATASCO</h3>
                        </div>
                    </div>
                    <div class="row mt-3 text-center">
                        <div class="col-4"><small class="text-muted">IZQ</small><br><span id="val-left" class="sensor-val">0</span>mm</div>
                        <div class="col-4"><small class="text-muted">FRENTE</small><br><span id="val-front" class="sensor-val">0</span>cm</div>
                        <div class="col-4"><small class="text-muted">DER</small><br><span id="val-right" class="sensor-val">0</span>mm</div>
                    </div>
                    <div class="d-flex justify-content-between align-items-center mt-3">
                        <small class="text-muted" id="err-log">Sistema OK</small>
                        <button class="btn btn-sm btn-light border" onclick="toggleView()">👁️ Ver Procesado</button>
                    </div>
                </div>
            </div>

            <div class="col-lg-5">
                <div class="card p-2 mb-3">
                    <div class="btn-group w-100" role="group">
                        <input type="radio" class="btn-check" name="btnradio" id="btn-manual" autocomplete="off" checked onclick="setMode('Manual')">
                        <label class="btn btn-outline-primary" for="btn-manual">🎮 Manual</label>
                        <input type="radio" class="btn-check" name="btnradio" id="btn-auto" autocomplete="off" onclick="setMode('Auto')">
                        <label class="btn btn-outline-primary" for="btn-auto">🤖 Autónomo</label>
                    </div>
                </div>

                <div class="card p-3" style="min-height: 400px;">
                    <ul class="nav nav-tabs mb-3" id="myTab" role="tablist">
                        <li class="nav-item"><a class="nav-link active" data-bs-toggle="tab" href="#tab-main">Control</a></li>
                        <li class="nav-item"><a class="nav-link" data-bs-toggle="tab" href="#tab-calib">Calibración</a></li>
                        <li class="nav-item"><a class="nav-link" data-bs-toggle="tab" href="#tab-diag">Hardware</a></li>
                    </ul>

                    <div class="tab-content">
                        <!-- TAB PRINCIPAL -->
                        <div class="tab-pane fade show active" id="tab-main">
                            <div id="panel-manual">
                                <label>Dirección <span class="val-badge" id="v-m-steer"></span></label>
                                <input type="range" class="form-range" min="40" max="100" id="man-steer" oninput="sendManual()">
                                <div class="text-center mb-3"><button class="btn btn-sm btn-light border" onclick="resetSteer()">Centrar</button></div>
                                <label>Motor <span class="val-badge" id="v-m-throt"></span></label>
                                <input type="range" class="form-range" min="-100" max="100" step="10" id="man-throt" oninput="sendManual()">
                                <button class="btn btn-danger w-100 mt-3 btn-custom" onclick="stopAll()">🛑 FRENO DE EMERGENCIA</button>
                            </div>
                            <div id="panel-auto" class="d-none">
                                <div class="text-center mb-4">
                                    <span id="status-badge" class="status-badge bg-stopped">DETENIDO</span>
                                    <div class="mt-2">Vueltas: <span class="h3 fw-bold" id="lap-count">0.00</span></div>
                                    <div class="mt-1 text-muted"><small id="dir-status">Esperando...</small></div>
                                </div>
                                <div class="d-grid gap-2">
                                    <button class="btn btn-success btn-custom" onclick="sendCommand('start')">▶ INICIAR CARRERA</button>
                                    <button class="btn btn-danger btn-custom" onclick="sendCommand('stop')">⏹ DETENER</button>
                                    <button class="btn btn-outline-secondary btn-sm mt-2" onclick="sendCommand('reset_laps')">↺ Reset Vueltas</button>
                                </div>
                            </div>
                        </div>

                        <!-- TAB CALIBRACIÓN -->
                        <div class="tab-pane fade" id="tab-calib">
                            <div style="height: 350px; overflow-y: auto; padding-right: 5px;">
                                <h6 class="text-primary mt-2">🏎️ Navegación (0-100)</h6>
                                <div class="mb-2"><label>Vel. Recta <span class="val-badge" id="v-spd"></span></label><input type="range" class="form-range" min="0" max="100" id="sl-spd" onchange="upd()"></div>
                                <div class="mb-2"><label>Vel. Reversa <span class="val-badge" id="v-rev-spd"></span></label><input type="range" class="form-range" min="0" max="100" id="sl-rev-spd" onchange="upd()"></div>
                                
                                <h6 class="text-primary mt-3">⏱️ Distancias & Tiempos</h6>
                                <div class="mb-2"><label>Dist. Frontal Choque (cm) <span class="val-badge" id="v-dist"></span></label><input type="range" class="form-range" min="10" max="60" id="sl-dist" onchange="upd()"></div>
                                <div class="mb-2"><label>Dist. Atasco Lateral (mm) <span class="val-badge" id="v-side-stuck"></span></label><input type="range" class="form-range" min="0" max="150" id="sl-side-stuck" onchange="upd()"></div>
                                <div class="mb-2"><label>Tiempo Reversa (s) <span class="val-badge" id="v-rev-time"></span></label><input type="range" class="form-range" min="0" max="3" step="0.1" id="sl-rev-time" onchange="upd()"></div>

                                <h6 class="text-primary mt-3">🎨 Colores (0-255)</h6>
                                <div class="mb-2"><label>Negro Máx <span class="val-badge" id="v-black"></span></label><input type="range" class="form-range" min="0" max="255" id="sl-black" onchange="upd()"></div>
                                <div class="mb-2"><label>Blanco Mín <span class="val-badge" id="v-white"></span></label><input type="range" class="form-range" min="0" max="255" id="sl-white" onchange="upd()"></div>
                                <div class="mb-2"><label>Sat Mín <span class="val-badge" id="v-sat"></span></label><input type="range" class="form-range" min="0" max="255" id="sl-sat" onchange="upd()"></div>
                                
                                <h6 class="text-primary mt-3">⚙️ Servo</h6>
                                <div class="mb-2"><label>Centro <span class="val-badge" id="v-center"></span></label><input type="range" class="form-range" min="50" max="90" id="sl-center" onchange="upd()"></div>
                                <div class="mb-2"><label>Izq (Min) <span class="val-badge" id="v-min"></span></label><input type="range" class="form-range" min="10" max="60" id="sl-min" onchange="upd()"></div>
                                <div class="mb-2"><label>Der (Max) <span class="val-badge" id="v-max"></span></label><input type="range" class="form-range" min="80" max="140" id="sl-max" onchange="upd()"></div>
                            </div>
                        </div>

                        <!-- TAB DIAGNOSTICO -->
                        <div class="tab-pane fade" id="tab-diag">
                            <div class="alert alert-light border">
                                <small class="d-block fw-bold text-muted">DRIVER MOTOR</small>
                                <h5 id="st-enable" class="text-success">ACTIVADO</h5>
                                <button class="btn btn-sm btn-outline-dark w-100 mt-1" onclick="sendCommand('toggle_enable')">Apagar/Encender</button>
                            </div>
                            <button class="btn btn-info w-100 shadow-sm" onclick="sendCommand('reset_sensors')">🔄 Reiniciar Sensores</button>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <script src="https://cdn.jsdelivr.net/npm/bootstrap@5.3.0/dist/js/bootstrap.bundle.min.js"></script>
    <script>
        let currentMode = 'Manual';
        
        setInterval(async () => {
            try {
                const res = await fetch('/api/status');
                const data = await res.json();
                const cfg = data.config;

                document.getElementById('val-front').innerText = data.sensor_front;
                document.getElementById('val-left').innerText = data.sensor_left;
                document.getElementById('val-right').innerText = data.sensor_right;
                document.getElementById('lap-count').innerText = data.lap_count.toFixed(2);
                document.getElementById('err-log').innerText = data.error_log;
                
                if(data.chosen_direction) {
                    document.getElementById('dir-status').innerText = "Giro Decidido: " + (data.chosen_direction == 'left' ? "IZQUIERDA" : "DERECHA");
                } else {
                    document.getElementById('dir-status').innerText = "Esperando decisión...";
                }

                if(data.visual_wall || data.sensor_front < cfg.min_front_dist || data.is_stuck) {
                    document.getElementById('alert-overlay').classList.remove('d-none');
                } else {
                    document.getElementById('alert-overlay').classList.add('d-none');
                }

                // Switch Indicator
                if(data.switch_active) {
                    document.getElementById('switch-indicator').classList.remove('d-none');
                } else {
                    document.getElementById('switch-indicator').classList.add('d-none');
                }

                const badge = document.getElementById('status-badge');
                if(data.status === 'Running') {
                    badge.innerText = "CORRIENDO"; badge.className = "status-badge bg-running";
                } else {
                    badge.innerText = "DETENIDO"; badge.className = "status-badge bg-stopped";
                }
                
                const enSpan = document.getElementById('st-enable');
                enSpan.innerText = data.motor_enable_state ? "ON" : "OFF";
                enSpan.className = data.motor_enable_state ? "fw-bold text-success" : "fw-bold text-danger";

                if(document.activeElement.tagName !== "INPUT" && document.activeElement.tagName !== "SELECT") {
                    sync('sl-spd', 'v-spd', cfg.base_speed);
                    sync('sl-rev-spd', 'v-rev-spd', cfg.reverse_speed);
                    
                    sync('sl-rev-time', 'v-rev-time', cfg.reverse_time);
                    sync('sl-dist', 'v-dist', cfg.min_front_dist);
                    sync('sl-side-stuck', 'v-side-stuck', cfg.side_stuck_dist);
                    
                    sync('sl-black', 'v-black', cfg.black_max_v);
                    sync('sl-white', 'v-white', cfg.white_min_v);
                    sync('sl-sat', 'v-sat', cfg.sat_min);
                    
                    sync('sl-center', 'v-center', cfg.servo_center);
                    sync('sl-min', 'v-min', cfg.servo_min);
                    sync('sl-max', 'v-max', cfg.servo_max);
                }
            } catch(e){}
        }, 500);

        function sync(idSl, idLbl, val) {
            const el = document.getElementById(idSl);
            if(el) el.value = val;
            const lbl = document.getElementById(idLbl);
            if(lbl) lbl.innerText = val;
        }

        async function setMode(m) {
            currentMode = m;
            document.getElementById('panel-manual').className = m === 'Manual' ? 'd-block' : 'd-none';
            document.getElementById('panel-auto').className = m === 'Auto' ? 'd-block' : 'd-none';
            await stopAll();
            await fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:'set_mode', mode:m})});
        }

        async function toggleView() { await fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:'toggle_view'})}); }
        async function sendCommand(c) { await fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:c})}); }
        async function stopAll() { document.getElementById('man-throt').value = 0; sendManual(); sendCommand('stop'); }
        async function testPulse(s) { await fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:'test_pulse', speed:s})}); }
        
        async function sendManual() {
            if(currentMode !== 'Manual') return;
            const angle = document.getElementById('man-steer').value;
            const speed = document.getElementById('man-throt').value;
            document.getElementById('v-m-steer').innerText = angle;
            document.getElementById('v-m-throt').innerText = speed;
            await fetch('/api/control', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({command:'manual', angle:parseInt(angle), speed:parseInt(speed)})});
        }
        function resetSteer() { document.getElementById('man-steer').value = 70; sendManual(); }

        async function upd() {
            const cfg = {
                base_speed: parseInt(document.getElementById('sl-spd').value),
                reverse_speed: parseInt(document.getElementById('sl-rev-spd').value),
                reverse_time: parseFloat(document.getElementById('sl-rev-time').value),
                min_front_dist: parseInt(document.getElementById('sl-dist').value),
                side_stuck_dist: parseInt(document.getElementById('sl-side-stuck').value),
                
                black_max_v: parseInt(document.getElementById('sl-black').value),
                white_min_v: parseInt(document.getElementById('sl-white').value),
                sat_min: parseInt(document.getElementById('sl-sat').value),
                
                servo_center: parseInt(document.getElementById('sl-center').value),
                servo_min: parseInt(document.getElementById('sl-min').value),
                servo_max: parseInt(document.getElementById('sl-max').value)
            };
            sync('sl-spd', 'v-spd', cfg.base_speed); 
            await fetch('/api/config', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(cfg)});
        }
    </script>
</body>
</html>
"""

# ==========================================
# 4. HARDWARE (SWITCH + SENSORES)
# ==========================================
class HardwareInterface:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(TRIG_FRONT, GPIO.OUT)
        GPIO.setup(ECHO_FRONT, GPIO.IN)
        GPIO.setup(XSHUT_LEFT, GPIO.OUT)
        GPIO.setup(XSHUT_RIGHT, GPIO.OUT)
        GPIO.setup(SERVO_PIN, GPIO.OUT)
        GPIO.setup(MOTOR_RPWM, GPIO.OUT)
        GPIO.setup(MOTOR_LPWM, GPIO.OUT)
        GPIO.setup(MOTOR_ENABLE, GPIO.OUT)
        
        # SWITCH DE COMPETENCIA
        GPIO.setup(SWITCH_PWR_PIN, GPIO.OUT)
        GPIO.output(SWITCH_PWR_PIN, GPIO.HIGH) # 3.3V constante
        GPIO.setup(SWITCH_READ_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        self.pwm_servo = GPIO.PWM(SERVO_PIN, 50)
        self.pwm_servo.start(0) 
        self.pwm_r = GPIO.PWM(MOTOR_RPWM, 1000)
        self.pwm_l = GPIO.PWM(MOTOR_LPWM, 1000)
        self.pwm_r.start(0)
        self.pwm_l.start(0)
        
        GPIO.output(MOTOR_ENABLE, GPIO.HIGH)
        
        self.tof_left = None; self.tof_right = None; self.i2c = None
        self.last_valid_dist = 400
        self.init_tof_sensors()
        self.servo_timer = None 

    def init_tof_sensors(self):
        print("🔧 Reiniciando Sensores I2C...")
        try:
            if self.i2c: self.i2c.deinit()
            time.sleep(0.2)
            self.i2c = busio.I2C(board.SCL, board.SDA)
            
            GPIO.output(XSHUT_LEFT, GPIO.LOW); GPIO.output(XSHUT_RIGHT, GPIO.LOW); time.sleep(0.1)
            
            GPIO.output(XSHUT_LEFT, GPIO.HIGH); time.sleep(0.1)
            self.tof_left = VL53L0X(self.i2c); self.tof_left.set_address(0x30)
            
            GPIO.output(XSHUT_RIGHT, GPIO.HIGH); time.sleep(0.1)
            self.tof_right = VL53L0X(self.i2c)
        except Exception as e:
            print(f"Error I2C: {e}")

    def read_ultrasonic(self):
        try:
            GPIO.output(TRIG_FRONT, True); time.sleep(0.00001); GPIO.output(TRIG_FRONT, False)
            start = time.time(); timeout = start + 0.04 
            while GPIO.input(ECHO_FRONT) == 0:
                start = time.time(); 
                if start > timeout: return 400
            stop = time.time()
            while GPIO.input(ECHO_FRONT) == 1:
                stop = time.time()
                if stop > timeout: return 400
            dist = ((stop - start) * 34300) / 2
            if dist > 500 or dist < 0: return 500
            return dist
        except: return 500

    def read_tof(self):
        try:
            l = self.tof_left.range if self.tof_left else 0
            r = self.tof_right.range if self.tof_right else 0
            if l == 0 and r == 0: self.init_tof_sensors(); return 2000, 2000
            if l > 2000: l = 2000
            if r > 2000: r = 2000
            return l, r
        except:
            self.init_tof_sensors()
            return 2000, 2000
    
    def check_switch(self):
        return GPIO.input(SWITCH_READ_PIN) == GPIO.HIGH

    def _disable_servo(self):
        self.pwm_servo.ChangeDutyCycle(0) 

    def set_actuators(self, angle, speed):
        # Anti-Temblor
        if angle != robot_state["servo_angle"]:
            duty = 2.5 + (angle / 18.0)
            self.pwm_servo.ChangeDutyCycle(duty)
            robot_state["servo_angle"] = int(angle)
            if self.servo_timer: self.servo_timer.cancel()
            self.servo_timer = threading.Timer(0.3, self._disable_servo)
            self.servo_timer.start()
        
        # Motor
        robot_state["motor_speed"] = speed
        if speed > 0:
            self.pwm_r.ChangeDutyCycle(speed); self.pwm_l.ChangeDutyCycle(0)
        elif speed < 0:
            self.pwm_r.ChangeDutyCycle(0); self.pwm_l.ChangeDutyCycle(abs(speed))
        else:
            self.pwm_r.ChangeDutyCycle(0); self.pwm_l.ChangeDutyCycle(0)

    def toggle_enable(self):
        st = not robot_state["motor_enable_state"]
        GPIO.output(MOTOR_ENABLE, GPIO.HIGH if st else GPIO.LOW)
        robot_state["motor_enable_state"] = st

hw = HardwareInterface()

# ==========================================
# 5. VISIÓN (BAJA LATENCIA + BLANCO PURO)
# ==========================================
class VisionSystem:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(3, FRAME_WIDTH)
        self.cap.set(4, FRAME_HEIGHT)
        self.last_frame_gray = None
        self.last_move_time = time.time()

    def process_frame(self):
        try:
            ret, frame = self.cap.read()
            if not ret: return None
            
            cfg = robot_state["config"]
            h_img, w_img = frame.shape[:2]
            
            y_start = int(h_img * cfg["roi_top"] / 100)
            y_end = int(h_img * cfg["roi_bottom"] / 100)
            if y_start >= y_end: y_start = 0; y_end = h_img
            
            mask_roi = np.zeros((h_img, w_img), dtype=np.uint8)
            mask_roi[y_start:y_end, :] = 255
            is_inside = (mask_roi == 255)
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            v_channel = hsv[:,:,2]
            s_channel = hsv[:,:,1]
            
            mask_black = (v_channel < cfg["black_max_v"]) & is_inside
            mask_white = (v_channel > cfg["white_min_v"]) & (s_channel < cfg["sat_min"]) & is_inside
            
            # Detección Pared Visual
            roi_h = y_end - y_start
            box_y = y_start + int(roi_h * 0.5)
            box_h = int(roi_h * 0.5)
            det_mask = mask_black[box_y : box_y+box_h, int(w_img*0.3) : int(w_img*0.7)]
            
            is_wall = False
            if det_mask.size > 0:
                if (np.count_nonzero(det_mask) / det_mask.size) > 0.4: is_wall = True
            robot_state["visual_wall"] = is_wall

            # VISUALIZACIÓN
            if robot_state["view_mode"] == 'debug':
                display = np.full_like(frame, 50) 
                display[mask_white] = [255, 255, 255]
                display[mask_black] = [0, 0, 0]
                cv2.rectangle(display, (0, y_start), (w_img, y_end), (0, 255, 0), 2)
                return cv2.imencode('.jpg', display)[1].tobytes()
            else:
                cv2.rectangle(frame, (0, y_start), (w_img, y_end), (0, 255, 0), 1)
                return cv2.imencode('.jpg', frame)[1].tobytes()
                
        except Exception as e: return None

vision = VisionSystem()

# ==========================================
# 6. BUCLE AUTÓNOMO (REACTIVO + SWITCH)
# ==========================================
def autonomous_loop():
    global last_movement_ts, tof_stuck_counter, last_tof_readings, sensor_reset_timer
    
    while True:
        try:
            time.sleep(0.05)
            
            # --- 1. LECTURA SWITCH COMPETENCIA ---
            is_switch_on = hw.check_switch()
            robot_state["switch_active"] = is_switch_on
            
            if is_switch_on:
                if robot_state["mode"] != "Auto" or robot_state["status"] != "Running":
                    print("⚠️ SWITCH ACTIVADO: Iniciando Carrera Autónoma")
                    robot_state["mode"] = "Auto"
                    robot_state["status"] = "Running"
                    robot_state["chosen_direction"] = None
            else:
                # Si el switch se apaga y estábamos corriendo por switch, parar.
                # (Pero permitir control manual si se activó por web)
                pass 

            # --- 2. LECTURA SENSORES ---
            dist_f = hw.read_ultrasonic()
            dist_l, dist_r = hw.read_tof()
            
            robot_state["sensor_front"] = int(dist_f)
            robot_state["sensor_left"] = int(dist_l)
            robot_state["sensor_right"] = int(dist_r)
            
            _ = vision.process_frame()
            cfg = robot_state["config"]

            # Auto-Reset Sensores periódico
            if (time.time() - sensor_reset_timer > 10) and robot_state["motor_speed"] == 0:
                hw.init_tof_sensors()
                sensor_reset_timer = time.time()

            if robot_state["mode"] != "Auto" or robot_state["status"] != "Running":
                if robot_state["mode"] == "Auto": hw.set_actuators(cfg["servo_center"], 0)
                continue

            # --- 3. LÓGICA DE ATASCOS ---
            
            # A) Atasco Lateral (Por Sensor)
            side_stuck = (dist_r < cfg["side_stuck_dist"]) or (dist_l < cfg["side_stuck_dist"])
            
            # B) Watchdog de Movimiento
            curr_readings = (dist_l, dist_r)
            diff_l = abs(curr_readings[0] - last_tof_readings[0])
            diff_r = abs(curr_readings[1] - last_tof_readings[1])
            
            if robot_state["motor_speed"] > 20:
                if diff_l < 5 and diff_r < 5: tof_stuck_counter += 1
                else: 
                    tof_stuck_counter = 0
                    last_movement_ts = time.time()
            else: last_movement_ts = time.time()
            last_tof_readings = curr_readings
            
            stuck_trigger = (tof_stuck_counter > 60) or (time.time() - last_movement_ts > 5.0)

            # --- 4. SECUENCIA DE EVASIÓN ---
            should_evade = False
            
            if dist_f < cfg["min_front_dist"] or robot_state["visual_wall"]: should_evade = True
            if side_stuck: should_evade = True; print("⚠️ Atasco Lateral Detectado")
            if stuck_trigger: should_evade = True; print("⚠️ Atasco por Tiempo Detectado")

            if should_evade:
                hw.set_actuators(cfg["servo_center"], 0)
                time.sleep(0.2)
                
                # Reversa
                hw.set_actuators(cfg["servo_center"], -int(cfg["reverse_speed"]))
                time.sleep(cfg["reverse_time"])
                
                # Decidir Dirección (Lado con más espacio)
                if robot_state["chosen_direction"] is None:
                    if dist_l > dist_r: robot_state["chosen_direction"] = 'left'
                    else: robot_state["chosen_direction"] = 'right'
                
                if robot_state["chosen_direction"] == 'left': turn_angle = cfg["servo_min"]
                else: turn_angle = cfg["servo_max"]
                
                # Girar y Avanzar
                hw.set_actuators(turn_angle, 0); time.sleep(0.2)
                hw.set_actuators(turn_angle, int(cfg["base_speed"]))
                time.sleep(0.6)
                
                # Reversa Contrarrotada si estaba atascado
                if stuck_trigger:
                    opp_angle = cfg["servo_max"] if turn_angle == cfg["servo_min"] else cfg["servo_min"]
                    hw.set_actuators(opp_angle, -int(cfg["reverse_speed"]))
                    time.sleep(0.4)
                    hw.set_actuators(turn_angle, int(cfg["base_speed"]))
                    time.sleep(0.4)

                tof_stuck_counter = 0
                last_movement_ts = time.time()
                continue

            # --- 5. NAVEGACIÓN RECTA ---
            target_speed = cfg["base_speed"]
            if dist_f < cfg["slow_dist"]: target_speed *= 0.6
            hw.set_actuators(cfg["servo_center"], int(target_speed))

        except Exception as e:
            robot_state["error_log"] = str(e)
            time.sleep(1)

t = threading.Thread(target=autonomous_loop)
t.daemon = True
t.start()

# ==========================================
# 7. API WEB
# ==========================================
app = Flask(__name__)

@app.route('/')
def index(): return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            frame = vision.process_frame()
            if frame: yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            else: time.sleep(0.1)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def status(): return jsonify(robot_state)

@app.route('/api/config', methods=['POST'])
def update_config():
    new_cfg = request.json
    robot_state["config"].update(new_cfg)
    save_config() 
    return jsonify({"msg": "Saved"})

@app.route('/api/control', methods=['POST'])
def api_control():
    data = request.json
    cmd = data.get('command')
    
    if cmd == 'set_mode':
        robot_state["mode"] = data.get('mode')
        hw.set_actuators(robot_state["config"]["servo_center"], 0)
        robot_state["status"] = "Stopped"
        
    elif cmd == 'toggle_view':
        robot_state["view_mode"] = 'debug' if robot_state["view_mode"] == 'normal' else 'normal'
        
    elif cmd == 'manual':
        hw.set_actuators(data.get('angle'), data.get('speed'))
        
    elif cmd == 'start': 
        robot_state["status"] = "Running"
        global last_movement_ts
        last_movement_ts = time.time()
        
    elif cmd == 'stop': 
        robot_state["status"] = "Stopped"
        hw.set_actuators(robot_state["config"]["servo_center"], 0)
    
    elif cmd == 'reset_laps': robot_state["lap_count"] = 0
    elif cmd == 'toggle_enable': hw.toggle_enable()
    elif cmd == 'reset_sensors': hw.init_tof_sensors()
    elif cmd == 'test_pulse':
        hw.set_actuators(robot_state["config"]["servo_center"], data.get('speed'))
        time.sleep(1.0)
        hw.set_actuators(robot_state["config"]["servo_center"], 0)

    return jsonify({"msg": "ok"})

if __name__ == '__main__':
    try: app.run(host='0.0.0.0', port=5000, debug=False)
    finally: hw.set_actuators(70, 0); GPIO.cleanup()
