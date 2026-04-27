from flask import Flask, Response, jsonify
from ultralytics import YOLO
import cv2
import numpy as np
import serial
import time
import os
import math
import threading
import json
import logging
import subprocess
import socket
import psutil
from concurrent.futures import ThreadPoolExecutor

# ========================================================
# STORICO VERSIONI
# ========================================================
# V30-V32: Implementazione Round-Robin e Profiler dei tempi.
# V33: Radar anti-paletto per saltare l'IA sui muri vuoti.
# V34.1: Distanza Euclidea, Maschere Vettoriali (SIMD) e valore cerchi a video.
# V34.4 (ATTUALE) - FIX FINALE & DEBUG COLORI: 
#  - Corretto bug critico di sovrascrittura cv2.VideoCapture in main_engine.
#  - Mostra a dashboard la stringa esatta dei 5 colori rilevati per permettere
#    un fine-tuning perfetto della calibrazione.
# ========================================================

# ========================================================
# 1. IMPOSTAZIONI PROCESSORE (Arduino UNO Q)
# ========================================================
os.environ["OMP_NUM_THREADS"] = "4"
os.environ["OPENCV_LOG_LEVEL"] = "SILENT"
cv2.ocl.setUseOpenCL(True)
yolo_device = 'cpu'

app = Flask(__name__)
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)
app.logger.disabled = True

# ========================================================
# 2. CONFIGURAZIONI COLORI E DATI WEB
# ========================================================
DEFAULT_COLORS = {
    "BLACK":  {"l": [0, 95],   "a": [0, 255],   "b": [0, 255],   "val": -2},
    "RED":    {"l": [95, 155], "a": [140, 255], "b": [120, 255], "val": -1},
    "YELLOW": {"l": [130, 255], "a": [100, 150], "b": [140, 255], "val": 0},
    "GREEN":  {"l": [50, 200], "a": [0, 120],   "b": [100, 200], "val": 1},
    "BLUE":   {"l": [30, 180], "a": [110, 180], "b": [0, 110],   "val": 2},
    "WHITE":  {"l": [180, 255], "a": [110, 145], "b": [110, 145], "val": -500}
}
DEFAULT_WB = {"b": 1.0, "g": 1.0, "r": 1.0}

def get_color_path(cam_id): return f"colori_{cam_id}.json"
def load_json(path, default_data):
    if not os.path.exists(path):
        with open(path, 'w') as f: json.dump(default_data, f, indent=4)
        return default_data
    with open(path, 'r') as f: return json.load(f)

target_colors = {'cam1': load_json(get_color_path("cam1"), DEFAULT_COLORS), 'cam2': load_json(get_color_path("cam2"), DEFAULT_COLORS)}
wb_gains = {'cam1': load_json("wb_cam1.json", DEFAULT_WB), 'cam2': load_json("wb_cam2.json", DEFAULT_WB)}

dati_web = {
    'cam1': {'info': "...", 'hex': "#555", 'fps': 0, 'ms': 0, 'yolo_status': "-", 'v_val': "-", 'v_colors': "-"},
    'cam2': {'info': "...", 'hex': "#555", 'fps': 0, 'ms': 0, 'yolo_status': "-", 'v_val': "-", 'v_colors': "-"},
    'cam1_targets': target_colors['cam1'], 'cam2_targets': target_colors['cam2'],
    'status': "Ready", 'sys': {'cpu': 0, 'ram_used': 0, 'ram_tot': 0, 'gpu': "N/A"}
}

frame_display, lock = None, threading.Lock()
fps_history = {'cam1': [], 'cam2': []}

# ========================================================
# 3. UTILITY HARDWARE
# ========================================================
def get_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except: return "127.0.0.1"

def get_adreno_gpu_load():
    try:
        with open('/sys/class/kgsl/kgsl-3d0/gpu_busy_percentage', 'r') as f:
            return f"{f.read().strip()} %"
    except:
        return "OCL ON" if cv2.ocl.useOpenCL() else "OFF"

def update_stats(cam_id, inference_time):
    now = time.time()
    fps_history[cam_id].append(now)
    if len(fps_history[cam_id]) > 20: fps_history[cam_id].pop(0)
    fps = len(fps_history[cam_id]) / (fps_history[cam_id][-1] - fps_history[cam_id][0]) if len(fps_history[cam_id]) > 1 else 0
    dati_web[cam_id].update({'fps': round(fps, 1), 'ms': round(inference_time * 1000, 1)})

def scan_cameras():
    cams = []
    try:
        out = subprocess.check_output(['v4l2-ctl', '--list-devices'], stderr=subprocess.STDOUT).decode('utf-8')
        blocks = out.split('\n\n')
        for block in blocks:
            if any(x in block.lower() for x in ['venus', 'decoder']): continue
            lines = block.strip().split('\n')
            if len(lines) > 1:
                name = lines[0].strip()
                for line in lines[1:]:
                    if '/dev/video' in line:
                        idx = int(line.strip().replace('/dev/video', ''))
                        print(f"   ✅ Trovata: {name} (ID: {idx})")
                        cams.append(idx)
                        break
    except: pass
    return sorted(list(set(cams)))[:2]

class CameraStream:
    def __init__(self, src, nome):
        self.src, self.nome = src, nome
        self.cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        threading.Thread(target=self.warmup, daemon=True).start()
        self.grabbed, self.frame = self.cap.read()
        self.stopped = False

    def warmup(self):
        time.sleep(2)
        os.system(f"v4l2-ctl -d /dev/video{self.src} -c white_balance_temperature_auto=0 -c exposure_auto=1 > /dev/null 2>&1")

    def start(self):
        threading.Thread(target=self.update, daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.cap.read()
            if ret: self.frame = frame
            self.grabbed = ret

    def read(self):
        if self.grabbed and self.frame is not None: return True, self.frame.copy()
        return False, None

# ========================================================
# 4. VISIONE AVANZATA (EUCLIDEA E MASCHERE)
# ========================================================
def apply_white_balance(frame, gains):
    b, g, r = cv2.split(frame)
    return cv2.merge((cv2.convertScaleAbs(b, alpha=gains['b']), 
                      cv2.convertScaleAbs(g, alpha=gains['g']), 
                      cv2.convertScaleAbs(r, alpha=gains['r'])))

def lab_to_hex(l, a, b):
    lab = np.array([[[l, a, b]]], dtype=np.uint8)
    bgr = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)[0][0]
    return '#%02x%02x%02x' % (bgr[2], bgr[1], bgr[0])

def classifica_colore_euclideo(l, a, b, target_dict):
    min_dist, best_val, best_name = float('inf'), 0, "UNKNOWN"
    for nome, d in target_dict.items():
        if nome == "WHITE": continue
        l_c, a_c, b_c = (d["l"][0]+d["l"][1])/2, (d["a"][0]+d["a"][1])/2, (d["b"][0]+d["b"][1])/2
        dist = math.sqrt((l - l_c)**2 + (a - a_c)**2 + (b - b_c)**2)
        if dist < min_dist: min_dist, best_val, best_name = dist, d["val"], nome
    return best_val, best_name

def analizza_bersaglio(img_lab, cx, cy, w_ell, h_ell, ang, target_dict):
    valori_anelli = []
    nomi_anelli = []
    prev_mask = np.zeros(img_lab.shape[:2], dtype=np.uint8)
    
    for scala in [0.2, 0.4, 0.6, 0.8, 1.0]:
        mask_c = np.zeros(img_lab.shape[:2], dtype=np.uint8)
        cv2.ellipse(mask_c, ((cx, cy), (w_ell * scala, h_ell * scala), ang), 255, -1)
        anello_mask = cv2.bitwise_xor(mask_c, prev_mask)
        prev_mask = mask_c
        
        media_lab = cv2.mean(img_lab, mask=anello_mask)[:3]
        if media_lab != (0.0, 0.0, 0.0):
            val, nome = classifica_colore_euclideo(media_lab[0], media_lab[1], media_lab[2], target_dict)
            valori_anelli.append(val)
            nomi_anelli.append(nome)
            
    sum_v = sum(valori_anelli) if valori_anelli else -999
    nomi_str = ", ".join(nomi_anelli) if nomi_anelli else "Nessuno"
    return sum_v, nomi_str

def process_circles_v34(frame, img_lab, target_dict):
    gray = cv2.medianBlur(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), 5)
    edges = cv2.Canny(gray, 40, 120)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    forme_sospette = False
    
    for c in sorted(cnts, key=cv2.contourArea, reverse=True)[:3]:
        if 800 < cv2.contourArea(c) < 30000:
            x_r, y_r, w_r, h_r = cv2.boundingRect(c)
            if 0.3 < (float(w_r)/max(h_r,1)) < 3.0:
                forme_sospette = True
                peri = cv2.arcLength(c, True)
                if peri > 0 and (4 * math.pi * (cv2.contourArea(c) / (peri * peri))) > 0.70 and len(c) >= 5:
                    (cx, cy), (w, h), ang = cv2.fitEllipse(c)
                    if (min(w,h)/max(w,h)) > 0.6:
                        sum_v, nomi_str = analizza_bersaglio(img_lab, cx, cy, w, h, ang, target_dict)
                        
                        cv2.putText(frame, f"VAL: {sum_v}", (int(cx-35), int(cy-30)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.ellipse(frame, ((cx, cy), (w, h), ang), (0, 255, 0), 2)
                        
                        if sum_v == 2: return frame, 'H', sum_v, nomi_str, False 
                        elif sum_v == 1: return frame, 'S', sum_v, nomi_str, False
                        elif sum_v == 0: return frame, 'U', sum_v, nomi_str, False
                        else:
                            return frame, None, sum_v, nomi_str, True
                            
    return frame, None, "-", "-", forme_sospette

# ========================================================
# 5. CORE ENGINE (IA + SERIAL)
# ========================================================
print("\n" + "="*50)
print("🚀 ARDUINO UNO Q - RESCUE MAZE AI V34.4 (Fix & Overlay)")
print("="*50)

print("\n🔍 Scansione Webcam in corso...")
cam_indices = scan_cameras()

print("🧠 Caricamento Rete Neurale YOLO...", end="", flush=True)
model = YOLO('best.onnx', task='detect')
print(" [OK]")

print("🎬 Avvio flussi video...")
# INIZIALIZZAZIONE CORRETTA DELLE TELECAMERE
c1 = CameraStream(cam_indices[0], "CAM 1").start() if len(cam_indices)>0 else None
c2 = CameraStream(cam_indices[1], "CAM 2").start() if len(cam_indices)>1 else None

ser = None

def get_serial():
    global ser
    if ser is None:
        for p in ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0']:
            if os.path.exists(p):
                try: ser = serial.Serial(p, 115200, timeout=0.05); break
                except: pass
    return ser

def analyze(frame, cam_id):
    if frame is None: return np.zeros((240, 320, 3), dtype=np.uint8), None
    t0, f_res = time.time(), cv2.resize(frame, (320, 240))
    h, w = f_res.shape[:2]
    cx, cy = w//2, h//2
    
    img_lab = cv2.cvtColor(f_res, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.mean(cv2.GaussianBlur(img_lab[cy-10:cy+10, cx-10:cx+10], (15,15), 0))[:3]
    _, col_name = classifica_colore_euclideo(l, a, b, target_colors[cam_id])
    dati_web[cam_id].update({'hex': '#%02x%02x%02x' % (cv2.cvtColor(np.array([[[l, a, b]]], dtype=np.uint8), cv2.COLOR_LAB2BGR)[0][0][2], 0, 0), 'info': f"{col_name} (L:{int(l)})"})

    # --- RADAR & VALORE CERCHI ---
    f_res, v_grezza, v_val, v_colors, serve_yolo = process_circles_v34(f_res, img_lab, target_colors[cam_id])
    dati_web[cam_id]['v_val'] = v_val
    dati_web[cam_id]['v_colors'] = v_colors

    if v_grezza is None and serve_yolo:
        dati_web[cam_id]['yolo_status'] = "ANALISI..."
        res = model(f_res, conf=0.5, verbose=False, device=yolo_device)
        if len(res[0].boxes) > 0:
            f_res = res[0].plot()
            cls = model.names[int(res[0].boxes[0].cls[0])].lower()
            v_grezza = 'H' if "phi" in cls else ('S' if "psi" in cls else 'U')
            dati_web[cam_id]['v_val'] = "Lettera"
            dati_web[cam_id]['v_colors'] = "-" 
    else:
        dati_web[cam_id]['yolo_status'] = "SKIP"

    cv2.drawMarker(f_res, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 1)
    
    update_stats(cam_id, time.time() - t0)
    return f_res, v_grezza

def main_engine():
    global frame_display, ser
    print(f"\n" + "-"*50)
    print(f"🌍 DASHBOARD WEB: http://{get_ip()}:5000/")
    print("-"*50 + "\n")

    cam_turn = 1
    p1 = p2 = np.zeros((240, 320, 3), dtype=np.uint8)

    while True:
        vittime = []
        if cam_turn == 1:
            if c1:
                _, f1 = c1.read()
                p1, v1 = analyze(f1, 'cam1')
                if v1: vittime.append(f"<1{v1}>")
            cam_turn = 2
        else:
            if c2:
                _, f2 = c2.read()
                p2, v2 = analyze(f2, 'cam2')
                if v2: vittime.append(f"<2{v2}>")
            cam_turn = 1

        s_act = get_serial()
        if vittime and s_act:
            for p in vittime: 
                try: s_act.write(p.encode()); s_act.flush()
                except: ser = None
        
        m = psutil.virtual_memory()
        dati_web['sys'] = {'cpu': psutil.cpu_percent(), 'ram_used': m.used//1024**2, 'ram_tot': m.total//1024**2, 'gpu': get_adreno_gpu_load()}
        with lock: frame_display = np.hstack((p1, p2))

threading.Thread(target=main_engine, daemon=True).start()

# ========================================================
# 6. DASHBOARD V34.4
# ========================================================
@app.route('/calibra_wb/<cam>', methods=['POST'])
def calibra_wb(cam):
    b, g, r = dati_web[cam]['raw_bgr']
    wb_gains[cam] = {"b": 240.0/max(b,1), "g": 240.0/max(g,1), "r": 240.0/max(r,1)}
    with open(get_wb_path(cam), 'w') as f: json.dump(wb_gains[cam], f, indent=4)
    return jsonify({"m": "White Balance calibrato!"})

@app.route('/auto_tara/<cam>', methods=['POST'])
def auto_tara(cam):
    target_colors[cam] = json.loads(json.dumps(STANDARD_CALIBRATION)) 
    with open(get_color_path(cam), 'w') as f: json.dump(target_colors[cam], f, indent=4)
    dati_web[f'{cam}_targets'] = target_colors[cam]
    return jsonify({"m": "Colori riportati ai valori predefiniti!"})

@app.route('/calibra/<cam>/<col>', methods=['POST'])
def calibra(cam, col):
    l, a, b = dati_web[cam]['raw']
    target_colors[cam][col] = {"l": [l-25, l+25], "a": [a-20, a+20], "b": [b-20, b+20], "val": DEFAULT_COLORS[col]["val"]}
    with open(get_color_path(cam), 'w') as f: json.dump(target_colors[cam], f, indent=4)
    dati_web[f'{cam}_targets'] = target_colors[cam]
    return jsonify({"m": f"Colore {col} clonato dalla telecamera e salvato!"})

@app.route('/video_feed')
def video_feed():
    def gen():
        while True:
            with lock:
                if frame_display is not None:
                    _, b = cv2.imencode('.jpg', frame_display, [cv2.IMWRITE_JPEG_QUALITY, 70])
                    yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + b.tobytes() + b'\r\n')
            time.sleep(0.04)
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/dati_sensori')
def data(): return jsonify(dati_web)

@app.route('/')
def index():
    return """
    <!DOCTYPE html><html><head><meta charset="utf-8"><title>AI Center V34.4</title>
    <style>
    body { background: #0a0a0a; color: #00ff00; font-family: monospace; text-align: center; margin: 0; }
    .header { background: #111; padding: 10px; border-bottom: 2px solid #00d2ff; }
    .stat-box { display: inline-block; padding: 5px 15px; border: 1px solid #00d2ff; margin: 5px; border-radius: 5px; }
    .vid { width: 95%; border: 2px solid #333; border-radius: 8px; margin-top: 10px; }
    .panel { background: #111; border: 1px solid #00d2ff; padding: 15px; width: 45%; border-radius: 10px; display: inline-block; vertical-align: top; margin: 10px; }
    .valore-cerchi { font-size: 1.5em; color: #ffeb3b; font-weight: bold; margin: 10px 0; border: 1px dashed #555; padding: 5px; }
    .colori-rilevati { font-size: 0.5em; color: #aaa; font-weight: normal; margin-top: 5px; display: block; }
    .highlight { color: #00d2ff; }
    button { padding: 8px 12px; margin: 2px; cursor: pointer; border-radius: 4px; border: none; font-weight: bold; }
    .WB { background: #fff; color: #000;} .TARA { background: #9b59b6; color: #fff; }
    .R { background: #e74c3c; color: #fff; } .G { background: #2ecc71; color: #fff; } .Y { background: #f1c40f; }
    .B { background: #3498db; color: #fff; } .K { background: #000; color: #fff; } .W { background: #fff; color: #000;}
    table { width: 100%; font-size: 0.8em; margin-top: 10px; border-collapse: collapse; }
    th, td { border: 1px solid #333; padding: 5px; }
    </style></head><body>
    <div class="header"><h1>RCJ MAZE 2026 - AI CENTER V34.4</h1><div id="st" style="color:#ffeb3b;">SCANSIONE...</div></div>
    <div class="stat-box">CPU: <span id="scpu" class="highlight">0</span>%</div>
    <div class="stat-box">RAM: <span id="sram" class="highlight">0</span> MB</div>
    <img src="/video_feed" class="vid">
    <div class="container">
        <div class="panel">
            <h3>CAM 1 (SINISTRA)</h3>
            <div id="f1" class="highlight">0 FPS</div>
            <div class="valore-cerchi">
                VALORE: <span id="v1">-</span>
                <span class="colori-rilevati" id="c1">-</span>
            </div>
            <div id="ys1" style="color:#ff5722;">IA: -</div>
            <button class="WB" onclick="act('calibra_wb/cam1')">WB</button><button class="TARA" onclick="act('auto_tara/cam1')">AUTO</button><br>
            <button class="R" onclick="cal('cam1','RED')">RED</button><button class="G" onclick="cal('cam1','GREEN')">GRN</button><button class="Y" onclick="cal('cam1','YELLOW')">YLW</button>
            <button class="B" onclick="cal('cam1','BLUE')">BLU</button><button class="K" onclick="cal('cam1','BLACK')">BLK</button><button class="W" onclick="cal('cam1','WHITE')">WHT</button>
            <div id="tg1"></div>
        </div>
        <div class="panel">
            <h3>CAM 2 (DESTRA)</h3>
            <div id="f2" class="highlight">0 FPS</div>
            <div class="valore-cerchi">
                VALORE: <span id="v2">-</span>
                <span class="colori-rilevati" id="c2">-</span>
            </div>
            <div id="ys2" style="color:#ff5722;">IA: -</div>
            <button class="WB" onclick="act('calibra_wb/cam2')">WB</button><button class="TARA" onclick="act('auto_tara/cam2')">AUTO</button><br>
            <button class="R" onclick="cal('cam2','RED')">RED</button><button class="G" onclick="cal('cam2','GREEN')">GRN</button><button class="Y" onclick="cal('cam2','YELLOW')">YLW</button>
            <button class="B" onclick="cal('cam2','BLUE')">BLU</button><button class="K" onclick="cal('cam2','BLACK')">BLK</button><button class="W" onclick="cal('cam2','WHITE')">WHT</button>
            <div id="tg2"></div>
        </div>
    </div>
    <script>
    setInterval(() => {
        fetch('/dati_sensori').then(r=>r.json()).then(d=>{
            document.getElementById('scpu').innerText = d.sys.cpu;
            document.getElementById('sram').innerText = d.sys.ram_used;
            document.getElementById('st').innerText = d.status;
            
            document.getElementById('f1').innerText = d.cam1.fps + " FPS | " + d.cam1.ms + " ms";
            document.getElementById('v1').innerText = d.cam1.v_val;
            document.getElementById('c1').innerText = d.cam1.v_colors;
            document.getElementById('ys1').innerText = "IA: " + d.cam1.yolo_status;
            
            document.getElementById('f2').innerText = d.cam2.fps + " FPS | " + d.cam2.ms + " ms";
            document.getElementById('v2').innerText = d.cam2.v_val;
            document.getElementById('c2').innerText = d.cam2.v_colors;
            document.getElementById('ys2').innerText = "IA: " + d.cam2.yolo_status;
            
            renderTable('tg1', d.cam1_targets); renderTable('tg2', d.cam2_targets);
        });
    }, 200);
    
    function renderTable(id, t) {
        let h = "<table><tr><th>COL</th><th>L</th><th>A</th><th>B</th></tr>";
        for(let k in t) if(t[k].l) h += `<tr><td>${k}</td><td>${t[k].l}</td><td>${t[k].a}</td><td>${t[k].b}</td></tr>`;
        document.getElementById(id).innerHTML = h + "</table>";
    }
    function act(u) { fetch('/'+u,{method:'POST'}).then(r=>r.json()).then(d=>alert(d.m)); }
    function cal(m,c) { fetch('/calibra/'+m+'/'+c,{method:'POST'}).then(r=>r.json()).then(d=>alert(d.m)); }
    </script></body></html>
    """

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000, threaded=True, use_reloader=False)