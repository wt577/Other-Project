import sys
import time
import math
import threading
import datetime
import numpy as np
import pandas as pd
import csv
import json
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from tkinter import filedialog
from tkinter.scrolledtext import ScrolledText
from pymavlink import mavutil
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib import style
import matplotlib.pyplot as plt

style.use('dark_background')
plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei'] 
plt.rcParams['axes.unicode_minus'] = False

class SafetyConfig:
    MAX_XY_TRAVEL = 1.0      
    MAX_Z_TRAVEL = 0.8       
    MAX_ATTITUDE_ANGLE = 30.0 
    MAX_PID = {'P': 20.0, 'I': 5.0, 'D': 2.0}
    MAX_TELEMETRY_ROWS = 100000  

class Theme:
    BG_APP = "#0b0f19"; BG_CARD = "#161b28"; BG_BOX = "#0d1424"       
    BG_INPUT = "#1c253c"; TEXT_MAIN = "#ffffff"; TEXT_SUB = "#8f9bb3"     
    COLOR_ROLL = "#00e5ff"; COLOR_PITCH = "#ffaa00"; COLOR_YAW = "#a64aff"    
    COLOR_THRUST = "#00e676"; COLOR_DANGER = "#ff3d71"; BORDER = "#222b45"
    BTN_DISABLED = "#2c364c" 

class DroneState:
    def __init__(self):
        self._lock = threading.Lock() 
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self.pos_x = 0.0; self.pos_y = 0.0; self.pos_z = 0.0 
        self.servo_outputs = [0] * 8
        self.connected = False; self.last_packet_time = 0.0
        self.target_system = 1; self.target_component = 1 
        self.flight_mode = "UNKNOWN"
        self.battery_voltage = 0.0; self.is_armed = False
        
        self.target_roll = 0.0; self.target_pitch = 0.0; self.target_yaw = 0.0; self.target_thrust = 0.0 
        self.test_target_x = 0.0; self.test_target_y = 0.0; self.test_target_z = 0.0
        self.current_control_mode = "ATTITUDE"
        
        self.send_enabled = False; self.maintain_offboard = False 
        self.ui_params = {}
        self.real_param_map = {} 
        self.param_updates = {}; self.log_queue = []
        self.initial_params_snapshot = {} 
        
        self.telemetry_data = []
        self.test_time_buf = []
        self.test_tgt1_buf = []; self.test_act1_buf = []
        self.test_tgt2_buf = []; self.test_act2_buf = []
        self.test_tgt3_buf = []; self.test_act3_buf = []
        self.is_step_testing = False

    def add_log(self, msg):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        with self._lock: 
            self.log_queue.append(f"[{ts}] {msg}")

    def update_param(self, k, v):
        with self._lock:
            ui_key = None
            if k == "KP_ROLL_ANGLE": ui_key = "Att_Roll_Out_P"
            elif k == "KP_ROLL_RATE": ui_key = "Att_Roll_In_P"
            elif k == "KI_ROLL_RATE": ui_key = "Att_Roll_In_I"
            elif k == "KD_ROLL_RATE": ui_key = "Att_Roll_In_D"
            elif k == "KP_PITCH_ANGLE": ui_key = "Att_Pitch_Out_P"
            elif k == "KP_PITCH_RATE": ui_key = "Att_Pitch_In_P"
            elif k == "KI_PITCH_RATE": ui_key = "Att_Pitch_In_I"
            elif k == "KD_PITCH_RATE": ui_key = "Att_Pitch_In_D"
            elif k == "KP_YAW_ANGLE": ui_key = "Att_Yaw_Out_P"
            elif k == "KP_YAW_RATE": ui_key = "Att_Yaw_In_P"
            elif k == "KI_YAW_RATE": ui_key = "Att_Yaw_In_I"
            elif k == "KD_YAW_RATE": ui_key = "Att_Yaw_In_D"
            elif k == "KPXP": ui_key = "Pos_X_Out_P"
            elif k == "KVXP": ui_key = "Pos_X_In_P"
            elif k == "KVXI": ui_key = "Pos_X_In_I"
            elif k == "KVXD": ui_key = "Pos_X_In_D"
            elif k == "KPYP": ui_key = "Pos_Y_Out_P"
            elif k == "KVYP": ui_key = "Pos_Y_In_P"
            elif k == "KVYI": ui_key = "Pos_Y_In_I"
            elif k == "KVYD": ui_key = "Pos_Y_In_D"
            elif k == "KPZP": ui_key = "Pos_Z_Out_P"
            elif k == "KVZP": ui_key = "Pos_Z_In_P"
            elif k == "KVZI": ui_key = "Pos_Z_In_I"
            elif k == "KVZD": ui_key = "Pos_Z_In_D"

            if ui_key:
                self.real_param_map[ui_key] = k
                if ui_key not in self.initial_params_snapshot and v != 0.0:
                    self.initial_params_snapshot[ui_key] = v
                self.ui_params[ui_key] = v
                self.param_updates[ui_key] = v

state = DroneState()

class MavlinkThread(threading.Thread):
    def __init__(self, local_port, target_ip, target_port):
        super().__init__()
        self.daemon = True
        self.stop_event = threading.Event()
        self.local_port = int(local_port)
        self.target_ip = target_ip
        self.target_port = int(target_port)
        self.master = None
        self.pending_params = {}
        self.param_lock = threading.Lock()

    def euler_to_quaternion(self, r, p, y):
        cr = math.cos(r * 0.5); sr = math.sin(r * 0.5); cp = math.cos(p * 0.5); sp = math.sin(p * 0.5); cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)
        return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]

    def run(self):
        try:
            conn_str = f'udpin:0.0.0.0:{self.local_port}'
            state.add_log(f"开始监听: {conn_str}")
            self.master = mavutil.mavlink_connection(conn_str)
            self.master.mav.srcComponent = 1
            self.master.mav.srcSystem = 255
            if hasattr(self.master, 'add_address'): 
                self.master.add_address(self.target_ip, self.target_port)
            self.request_data_stream()
            threading.Timer(2.0, self.request_pid_params).start()
        except Exception as e:
            state.add_log(f"通信初始化失败: {e}")
            return

        last_hb = 0; last_send = 0; mode_chk = 0

        while not self.stop_event.is_set():
            try:
                while True:
                    msg = self.master.recv_match(blocking=False)
                    if not msg: break 

                    state.last_packet_time = time.time(); state.connected = True
                    mtype = msg.get_type()
                  
                    if mtype == 'HEARTBEAT':
                        if state.target_system != msg.get_srcSystem():
                            state.target_system = msg.get_srcSystem()
                            state.target_component = msg.get_srcComponent()
                            state.add_log(f"已锁定目标: SysID={state.target_system}")
                            threading.Thread(target=self.request_pid_params).start()
                        try:
                            if msg.custom_mode == 6: state.flight_mode = "OFFBOARD"
                            elif msg.custom_mode == 1: state.flight_mode = "MANUAL"
                            else: state.flight_mode = f"Mode_{msg.custom_mode}"
                        except: state.flight_mode = f"Mode_{msg.custom_mode}"
                        state.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0

                    elif mtype == 'ATTITUDE': 
                        state.roll, state.pitch, state.yaw = math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)
                    elif mtype == 'LOCAL_POSITION_NED': 
                        state.pos_x, state.pos_y, state.pos_z = msg.x, msg.y, msg.z 
                    elif mtype == 'SERVO_OUTPUT_RAW': 
                        state.servo_outputs[0:4] = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
                    elif mtype == 'SYS_STATUS': 
                        state.battery_voltage = msg.voltage_battery / 1000.0
                    elif mtype == 'PARAM_VALUE': 
                        state.update_param(msg.param_id, msg.param_value)
              
                now = time.time()
                if now - last_hb > 1.0: 
                    self.master.mav.heartbeat_send(6, 8, 0, 0, 0)
                    last_hb = now
              
                if state.send_enabled and (now - last_send > 0.05):

                    if not (state.is_step_testing and state.current_control_mode == 'POSITION'):
                        q = self.euler_to_quaternion(math.radians(state.target_roll), math.radians(state.target_pitch), math.radians(state.target_yaw))
                        self.master.mav.set_attitude_target_send(0, state.target_system, state.target_component, 0b00000111, q, 0, 0, 0, state.target_thrust)
                    else:
                        # 只有在位置环阶跃测试的那 3 秒内，才会瞬间发送真正的 NED 位置指令
                        self.master.mav.set_position_target_local_ned_send(
                            0, state.target_system, state.target_component, 1, 0b0000111111111000, 
                            state.test_target_x, state.test_target_y, state.test_target_z, 0, 0, 0, 0, 0, 0, 0, 0)
                    
                    last_send = now
                    
                    if state.maintain_offboard and (now - mode_chk > 0.5):
                        if state.flight_mode != "OFFBOARD": self.force_offboard()
                        mode_chk = now
              
                self._process_param_queue()
                time.sleep(0.005) 
            except Exception as e:
                time.sleep(0.01)

    def request_data_stream(self):
        if self.master:
            for s in [0, 3, 10, 11, 12, 6]: self.master.mav.request_data_stream_send(state.target_system, state.target_component, s, 4, 1)

    def request_pid_params(self):
        if not self.master: return
        keys = ["KP_ROLL_ANGLE", "KP_ROLL_RATE", "KI_ROLL_RATE", "KD_ROLL_RATE", 
                "KP_PITCH_ANGLE", "KP_PITCH_RATE", "KI_PITCH_RATE", "KD_PITCH_RATE",
                "KP_YAW_ANGLE", "KP_YAW_RATE", "KI_YAW_RATE", "KD_YAW_RATE",
                "KPXP", "KVXP", "KVXI", "KVXD", "KPYP", "KVYP", "KVYI", "KVYD",
                "KPZP", "KVZP", "KVZI", "KVZD"]
        for p in keys:
            self.master.mav.param_request_read_send(state.target_system, state.target_component, p.encode('utf-8'), -1)
            time.sleep(0.01)

    def set_param_async(self, name, value):
        with self.param_lock: self.pending_params[name] = value

    def _process_param_queue(self):
        item = None
        with self.param_lock:
            if self.pending_params:
                k = next(iter(self.pending_params))
                v = self.pending_params.pop(k)
                item = (k, v)
        if item and self.master:
            self.master.mav.param_set_send(state.target_system, state.target_component, item[0].encode('utf-8'), item[1], mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def handle_arm_switch(self, enable):
        if self.master:
            if enable:
                state.maintain_offboard = True
                state.add_log("流程：进入 OFFBOARD -> 解锁")
                self.force_offboard()
                time.sleep(0.2)
                self.send_command_arm(True)
            else:
                state.maintain_offboard = False
                state.add_log("流程：切回 MANUAL -> 上锁")
                self.force_manual_mode()
                time.sleep(0.1)
                self.send_command_arm(False)

    def send_command_arm(self, arm):
        self.master.mav.command_long_send(state.target_system, state.target_component, 400, 0, 1 if arm else 0, 0, 0, 0, 0, 0, 0)
    def force_offboard(self):
        if self.master: self.master.mav.command_long_send(state.target_system, state.target_component, 176, 0, 1, 6, 0, 0, 0, 0, 0)
    def force_manual_mode(self):
        if self.master: self.master.mav.command_long_send(state.target_system, state.target_component, 176, 0, 1, 1, 0, 0, 0, 0, 0)
    def stop(self):
        self.stop_event.set()
        if self.master: self.master.close()


class DroneApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Quad  PID 自整定实验系统")
        self.root.geometry("1800x1000")
        self.root.configure(bg=Theme.BG_APP)
        self.comm = None
        self.active_tab = 'console'
        self.plot_counter = 0
        self.manual_entries = {}
        
        self.ai_df = None
        self.ai_fs = 50.0 
        self.ai_history_params = [] 
        self.ai_entries = {'p1': [], 'p2': [], 'p3': []}
        self.ai_p1_names = []; self.ai_p2_names = []; self.ai_p3_names = []

        self.build_navigation()
        self.main_container = tk.Frame(self.root, bg=Theme.BG_APP)
        self.main_container.pack(fill=tk.BOTH, expand=True)
        
        self.pages = {
            'console': tk.Frame(self.main_container, bg=Theme.BG_APP), 
            'tune_collect': tk.Frame(self.main_container, bg=Theme.BG_APP),
            'tune_ai': tk.Frame(self.main_container, bg=Theme.BG_APP)
        }
        
        self.build_page_console(self.pages['console'])
        self.build_page_tune_collect(self.pages['tune_collect'])
        self.build_page_tune_ai(self.pages['tune_ai'])
        
        self.switch_page('console')
        self.update_loop()

    def create_card(self, parent, title="", pad=8): 
        outer = tk.Frame(parent, bg=Theme.BORDER, padx=1, pady=1)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=4) 
        inner = tk.Frame(outer, bg=Theme.BG_CARD, padx=pad, pady=pad)
        inner.pack(fill=tk.BOTH, expand=True)
        
        if title == "系统日志":
            h_f = tk.Frame(inner, bg=Theme.BG_CARD)
            h_f.pack(fill=tk.X, pady=(0, 2))
            tk.Label(h_f, text=title, bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, font=("微软雅黑", 10, "bold")).pack(side=tk.LEFT)
            tk.Button(h_f, text="🗑️ 清空", bg=Theme.BG_INPUT, fg=Theme.TEXT_SUB, bd=0, font=("微软雅黑", 8), cursor="hand2", command=self.clear_logs).pack(side=tk.RIGHT)
        elif title: 
            tk.Label(inner, text=title, bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, font=("微软雅黑", 10, "bold")).pack(anchor='w', pady=(0, 2))
        return inner

    def clear_logs(self):
        self.log_text.delete("1.0", tk.END)
        with state._lock: state.log_queue.clear()

    def build_navigation(self):
        nav = tk.Frame(self.root, bg=Theme.BG_APP, height=45)
        nav.pack(side=tk.TOP, fill=tk.X); nav.pack_propagate(False)
        tk.Label(nav, text="❖ 台架与PID整定实验控制台", font=("微软雅黑", 13, "bold"), fg=Theme.COLOR_ROLL, bg=Theme.BG_APP).pack(side=tk.LEFT, padx=20)
        btn_box = tk.Frame(nav, bg=Theme.BG_APP); btn_box.pack(side=tk.LEFT, padx=40)
        self.nav_btns = {}
        
        tabs = [('console', "⛐ 主控台 (连接与飞行)"), ('tune_collect', "⚙ 智能实验与采集"), ('tune_ai', "🧠 离线 PID自整定")]
        for key, text in tabs:
            btn = tk.Button(btn_box, text=text, font=("微软雅黑", 10, "bold"), bg=Theme.BG_APP, fg=Theme.TEXT_SUB, bd=0, activebackground=Theme.BG_CARD, activeforeground=Theme.COLOR_ROLL, cursor="hand2", command=lambda k=key: self.switch_page(k))
            btn.pack(side=tk.LEFT, padx=10, pady=5); self.nav_btns[key] = btn
            
        tk.Button(nav, text="↓ 导出数据", font=("微软雅黑", 10, "bold"), bg="#00bcd4", fg="#000", bd=0, cursor="hand2", padx=10, pady=2, command=self.export_csv).pack(side=tk.RIGHT, padx=30, pady=8)
        tk.Frame(self.root, bg=Theme.BORDER, height=1).pack(fill=tk.X)

    def switch_page(self, target_key):
        self.active_tab = target_key
        for k, btn in self.nav_btns.items(): btn.config(fg=Theme.COLOR_ROLL if k == target_key else Theme.TEXT_SUB)
        for p in self.pages.values(): p.pack_forget()
        self.pages[target_key].pack(fill=tk.BOTH, expand=True)

    # ========================== 页面1: 主控台 ==========================
    def build_page_console(self, page):
        left = tk.Frame(page, bg=Theme.BG_APP, width=320)
        left.pack(side=tk.LEFT, fill=tk.Y, pady=2); left.pack_propagate(False) 
        
        c_stat = self.create_card(left)
        top_bar = tk.Frame(c_stat, bg=Theme.BG_CARD); top_bar.pack(fill=tk.X)
        tk.Label(top_bar, text="◷ 通信状态监控", fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("微软雅黑", 9, "bold")).pack(side=tk.LEFT)
        self.lbl_latency_top = tk.Label(top_bar, text="-- ms", bg="#0a3a40", fg=Theme.COLOR_ROLL, font=("Consolas", 8, "bold"), padx=8); self.lbl_latency_top.pack(side=tk.RIGHT)
        box = tk.Frame(c_stat, bg=Theme.BG_BOX, highlightbackground=Theme.BORDER, highlightthickness=1); box.pack(fill=tk.X, pady=4)
        inner_f = tk.Frame(box, bg=Theme.BG_BOX); inner_f.pack(fill=tk.X, padx=10, pady=8)
        self.cv_dot = tk.Canvas(inner_f, width=12, height=12, bg=Theme.BG_BOX, highlightthickness=0); self.cv_dot.pack(side=tk.LEFT, padx=(0, 8))
        self.dot_id = self.cv_dot.create_oval(1, 1, 11, 11, fill=Theme.COLOR_DANGER, outline="")
        link_text_f = tk.Frame(inner_f, bg=Theme.BG_BOX); link_text_f.pack(side=tk.LEFT, fill=tk.X)
        self.lbl_big_link = tk.Label(link_text_f, text="离线 (OFFLINE)", font=("微软雅黑", 11, "bold"), fg=Theme.COLOR_DANGER, bg=Theme.BG_BOX); self.lbl_big_link.pack(anchor='w')
        self.lbl_ip_info = tk.Label(link_text_f, text="--:--", fg=Theme.TEXT_SUB, bg=Theme.BG_BOX, font=("Consolas", 8)); self.lbl_ip_info.pack(anchor='w')
        
        grid_f = tk.Frame(c_stat, bg=Theme.BG_CARD); grid_f.pack(fill=tk.X, pady=2)
        def add_sys_info(r, c, title, attr):
            f = tk.Frame(grid_f, bg=Theme.BG_CARD); f.grid(row=r, column=c, sticky="w", padx=15, pady=2)
            tk.Label(f, text=title, fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("Consolas", 7, "bold")).pack(anchor='w')
            lbl = tk.Label(f, text="--", fg=Theme.TEXT_MAIN, bg=Theme.BG_CARD, font=("Consolas", 10, "bold")); lbl.pack(anchor='w'); setattr(self, attr, lbl)
            
        add_sys_info(0, 0, "SYSTEM ID", "lbl_sysid"); add_sys_info(0, 1, "MODEL", "lbl_model")
        add_sys_info(1, 0, "BATTERY", "lbl_bat"); add_sys_info(1, 1, "LATENCY", "lbl_lat_bot")

        c_param = self.create_card(left, "UDP 节点配置")
        def add_input(parent, label, default, attr_name):
            tk.Label(parent, text=label, fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("Consolas", 8)).pack(anchor='w')
            ent = tk.Entry(parent, bg=Theme.BG_INPUT, fg=Theme.TEXT_MAIN, bd=0, insertbackground=Theme.COLOR_ROLL, font=("Consolas", 10))
            ent.insert(0, default); ent.pack(fill=tk.X, ipady=3, pady=(0,4)); setattr(self, attr_name, ent)
            
        add_input(c_param, "本地端口 (LOCAL PORT)", "14550", "ent_local")
        add_input(c_param, "目标 IP (TARGET IP)", "192.168.4.1", "ent_ip")
        add_input(c_param, "目标端口 (TARGET PORT)", "14555", "ent_port")
        self.btn_conn = tk.Button(c_param, text="启 动 连 接", bg=Theme.COLOR_DANGER, fg=Theme.TEXT_MAIN, font=("微软雅黑", 10, "bold"), bd=0, command=self.toggle_conn)
        self.btn_conn.pack(fill=tk.X, pady=(2, 0), ipady=4)

        c_ctrl = self.create_card(left, "控制权限")
        self.is_send_enabled = False
        self.btn_send = tk.Button(c_ctrl, text="[1] 开启指令发送", font=("微软雅黑", 10, "bold"), bg=Theme.BTN_DISABLED, fg="#888", bd=0, cursor="hand2", command=self.toggle_send_button, state="disabled")
        self.btn_send.pack(fill=tk.X, pady=(4, 8), ipady=4)
        self.is_arming_requested = False
        self.btn_arm = tk.Button(c_ctrl, text="[2] 允许解锁电机 (ARM)", font=("微软雅黑", 10, "bold"), bg=Theme.BTN_DISABLED, fg="#888", bd=0, cursor="hand2", command=self.toggle_arm_button, state="disabled")
        self.btn_arm.pack(fill=tk.X, pady=(0, 4), ipady=4)

        c_setpoint = self.create_card(left, "飞行控制(SETPOINTS)")
        self.create_canvas_slider(c_setpoint, "Roll", -30, 30, 'roll', Theme.COLOR_ROLL)
        self.create_canvas_slider(c_setpoint, "Pitch", -30, 30, 'pitch', Theme.COLOR_PITCH)
        self.create_canvas_slider(c_setpoint, "Yaw", -180, 180, 'yaw', Theme.COLOR_YAW)
        self.create_canvas_slider(c_setpoint, "THR(推力)", 1100, 1900, 'thrust', Theme.COLOR_THRUST)

        right = tk.Frame(page, bg=Theme.BG_APP)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=2)
        top_f = tk.Frame(right, bg=Theme.BG_APP, height=80); top_f.pack(fill=tk.X); top_f.pack_propagate(False) 
        self.lbl_v_roll = self.create_metric_card(top_f, "ROLL", Theme.COLOR_ROLL)
        self.lbl_v_pitch = self.create_metric_card(top_f, "PITCH", Theme.COLOR_PITCH)
        self.lbl_v_yaw = self.create_metric_card(top_f, "YAW", Theme.COLOR_YAW)

        mid_f = tk.Frame(right, bg=Theme.BG_APP); mid_f.pack(fill=tk.BOTH, expand=True)
        self.setup_2x2_charts(self.create_card(mid_f, "Telemetry 动态波形"))

        btm_show_f = tk.Frame(right, bg=Theme.BG_APP, height=260); btm_show_f.pack(fill=tk.X); btm_show_f.pack_propagate(False)
        pwm_f = tk.Frame(btm_show_f, bg=Theme.BG_APP); pwm_f.place(relx=0, rely=0, relwidth=0.45, relheight=1.0)
        c_pwm = self.create_card(pwm_f, "电机 PWM 输出"); self.pwm_bars = []
        for i, color in enumerate([Theme.COLOR_ROLL, Theme.COLOR_PITCH, Theme.COLOR_YAW, Theme.COLOR_THRUST]):
            row = tk.Frame(c_pwm, bg=Theme.BG_CARD); row.pack(fill=tk.X, pady=7) 
            tk.Label(row, text=f"M{i+1}", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("Consolas", 9, "bold"), width=3).pack(side=tk.LEFT)
            cv = tk.Canvas(row, height=12, bg=Theme.BG_INPUT, highlightthickness=0); cv.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
            cv.create_rectangle(0, 0, 9999, 12, fill="#1c253c", outline=""); rect = cv.create_rectangle(0, 0, 0, 12, fill=color, outline="")
            lbl = tk.Label(row, text="1000", bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, font=("Consolas", 10), width=4); lbl.pack(side=tk.RIGHT)
            self.pwm_bars.append((cv, rect, lbl))

        att_f = tk.Frame(btm_show_f, bg=Theme.BG_APP); att_f.place(relx=0.45, rely=0, relwidth=0.55, relheight=1.0)
        self.cv_att = tk.Canvas(self.create_card(att_f, "姿态球(Attitude Indicator)"), bg=Theme.BG_CARD, highlightthickness=0)
        self.cv_att.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.cv_att.bind("<Configure>", lambda e: self.draw_attitude(self.cv_att, state.roll, state.pitch))

        log_f = tk.Frame(right, bg=Theme.BG_APP, height=100); log_f.pack(fill=tk.X); log_f.pack_propagate(False) 
        self.log_text = ScrolledText(self.create_card(log_f, "系统日志", pad=5), bg=Theme.BG_INPUT, fg=Theme.TEXT_SUB, bd=0, font=("Consolas", 8))
        self.log_text.pack(fill=tk.BOTH, expand=True)

    # ========================== 页面2: 智能采集 ==========================
    def build_page_tune_collect(self, page):
        left = tk.Frame(page, bg=Theme.BG_APP, width=340); left.pack(side=tk.LEFT, fill=tk.Y, pady=10)
        right = tk.Frame(page, bg=Theme.BG_APP); right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=10)

        c_mode = self.create_card(left, "◎ 切换当前实验类型", pad=12)
        self.loop_mode_var = tk.StringVar(value="ATTITUDE")
        tk.Radiobutton(c_mode, text="实验1：姿态环实验 (角度/角速率)", variable=self.loop_mode_var, value="ATTITUDE", bg=Theme.BG_CARD, fg=Theme.COLOR_ROLL, selectcolor=Theme.BG_APP, font=("微软雅黑", 10, "bold"), command=self.switch_tune_mode).pack(anchor='w', pady=5)
        tk.Radiobutton(c_mode, text="实验2：位置环实验 (位置/速度)", variable=self.loop_mode_var, value="POSITION", bg=Theme.BG_CARD, fg=Theme.COLOR_PITCH, selectcolor=Theme.BG_APP, font=("微软雅黑", 10, "bold"), command=self.switch_tune_mode).pack(anchor='w', pady=5)

        c_at = self.create_card(left, "⚡ 闭环阶跃测试与数据采集", pad=12)
        tk.Label(c_at, text="阶跃目标轴:", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB).pack(anchor='w', pady=(5,5))
        
        self.axis_frame = tk.Frame(c_at, bg=Theme.BG_CARD); self.axis_frame.pack(fill=tk.X)
        self.tune_axis_var = tk.StringVar(value="Roll")
        
        self.rb_axis1 = tk.Radiobutton(self.axis_frame, text="Roll", variable=self.tune_axis_var, value="Roll", bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, selectcolor=Theme.BG_APP)
        self.rb_axis2 = tk.Radiobutton(self.axis_frame, text="Pitch", variable=self.tune_axis_var, value="Pitch", bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, selectcolor=Theme.BG_APP)
        self.rb_axis3 = tk.Radiobutton(self.axis_frame, text="Yaw", variable=self.tune_axis_var, value="Yaw", bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, selectcolor=Theme.BG_APP)
        self.rb_axis4 = tk.Radiobutton(self.axis_frame, text="All", variable=self.tune_axis_var, value="All", bg=Theme.BG_CARD, fg=Theme.COLOR_THRUST, selectcolor=Theme.BG_APP)
        self.rb_axis1.pack(side=tk.LEFT, expand=True); self.rb_axis2.pack(side=tk.LEFT, expand=True)
        self.rb_axis3.pack(side=tk.LEFT, expand=True); self.rb_axis4.pack(side=tk.LEFT, expand=True)

        tk.Label(c_at, text="相对阶跃幅度 (姿态:度 | 位置:米):", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB).pack(anchor='w', pady=(15,5))
        self.ent_step_amp = tk.Entry(c_at, bg=Theme.BG_INPUT, fg=Theme.TEXT_MAIN, bd=0, font=("Consolas", 11)); self.ent_step_amp.insert(0, "15"); self.ent_step_amp.pack(fill=tk.X, ipady=5)
        
        tk.Frame(c_at, bg=Theme.BORDER, height=1).pack(fill=tk.X, pady=15)
        self.btn_step_test = tk.Button(c_at, text="▶ 安全运行阶跃测试", bg=Theme.COLOR_THRUST, fg="#000", font=("微软雅黑", 11, "bold"), bd=0, cursor="hand2", command=self.run_step_test_thread)
        self.btn_step_test.pack(fill=tk.X, pady=5, ipady=6)
        self.btn_export_test = tk.Button(c_at, text="💾 导出三轴大一统数据", bg="#00bcd4", fg="#000", font=("微软雅黑", 10, "bold"), bd=0, cursor="hand2", command=self.export_test_data)
        self.btn_export_test.pack(fill=tk.X, pady=(10,0), ipady=4)

        c_perf = self.create_card(left, "∿ 主轴性能指标")
        perf_grid = tk.Frame(c_perf, bg=Theme.BG_CARD); perf_grid.pack(fill=tk.BOTH, expand=True)
        self.perf_labels = {}
        for i, (title, unit) in enumerate([("超调量", "%"), ("上升时间", "ms"), ("稳态误差", "")]):
            f = tk.Frame(perf_grid, bg=Theme.BG_BOX, padx=10, pady=12); f.grid(row=0, column=i, padx=5, pady=5, sticky='nsew')
            tk.Label(f, text=title, bg=Theme.BG_BOX, fg=Theme.TEXT_SUB, font=("微软雅黑", 8)).pack()
            lbl = tk.Label(f, text="-", bg=Theme.BG_BOX, fg=Theme.COLOR_ROLL, font=("Consolas", 14, "bold")); lbl.pack(pady=4)
            self.perf_labels[title] = lbl

        c_chart = self.create_card(right, "相对物理响应曲线")
        self.fig_tune = Figure(figsize=(8, 2.5), facecolor=Theme.BG_CARD); self.ax_tune = self.fig_tune.add_subplot(111, facecolor=Theme.BG_APP)
        self.ax_tune.grid(True, color=Theme.BORDER, linestyle='--', alpha=0.5); self.ax_tune.tick_params(colors=Theme.TEXT_SUB, labelsize=8)
        self.line_target, = self.ax_tune.plot([], [], color=Theme.TEXT_SUB, linestyle=':', label="相对期望增量")
        self.line_act1, = self.ax_tune.plot([], [], color=Theme.COLOR_ROLL, lw=2, label="轴1 (Roll/X) 物理响应")
        self.line_act2, = self.ax_tune.plot([], [], color=Theme.COLOR_PITCH, lw=2, label="轴2 (Pitch/Y) 物理响应")
        self.line_act3, = self.ax_tune.plot([], [], color=Theme.COLOR_YAW, lw=2, label="轴3 (Yaw/Z) 物理响应")
        self.ax_tune.legend(facecolor=Theme.BG_CARD, labelcolor=Theme.TEXT_MAIN, fontsize=8)
        self.canvas_tune = FigureCanvasTkAgg(self.fig_tune, master=c_chart); self.canvas_tune.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.c_man = self.create_card(right, "📝 参数覆写")
        self.param_container = tk.Frame(self.c_man, bg=Theme.BG_CARD); self.param_container.pack(fill=tk.X, expand=True)
        bot_btn_f = tk.Frame(self.c_man, bg=Theme.BG_CARD); bot_btn_f.pack(fill=tk.X, pady=(15, 5), padx=10)
        tk.Button(bot_btn_f, text="写入全组参数", bg=Theme.COLOR_ROLL, fg=Theme.BG_APP, font=("微软雅黑", 10, "bold"), bd=0, padx=15, pady=4, cursor="hand2", command=self.write_manual_params).pack(side=tk.LEFT, padx=(0, 10))
        tk.Button(bot_btn_f, text="刷新读取飞控", bg=Theme.BG_INPUT, fg=Theme.TEXT_SUB, font=("微软雅黑", 10), bd=0, padx=15, pady=4, cursor="hand2", command=self.refresh_pids).pack(side=tk.LEFT)

        self.switch_tune_mode()

    # ========================== 页面3: 离线 AI 自整定 ==========================
    def build_page_tune_ai(self, page):
        top_f = tk.Frame(page, bg="#161b28", pady=8, highlightbackground="#222b45", highlightthickness=1); top_f.pack(fill=tk.X, padx=10, pady=10)
        btn_box = tk.Frame(top_f, bg="#161b28"); btn_box.pack(side=tk.LEFT, padx=10)
        tk.Button(btn_box, text="📂 导入响应 CSV", font=("微软雅黑", 10, "bold"), bg="#00bcd4", fg="black", command=self.ai_load_csv).pack(side=tk.TOP, fill=tk.X, pady=2)
        tk.Button(btn_box, text="↩ 撤销回滚 (Undo)", font=("微软雅黑", 9), bg="#444", fg="white", command=self.ai_rollback_params).pack(side=tk.TOP, fill=tk.X)

        type_f = tk.Frame(top_f, bg="#161b28"); type_f.pack(side=tk.LEFT, padx=20)
        tk.Label(type_f, text="目标环:", bg="#161b28", fg="white", font=("微软雅黑", 9)).pack(anchor='w')
        self.ai_exp_type = tk.StringVar(value="ATTITUDE")
        tk.Radiobutton(type_f, text="姿态环", variable=self.ai_exp_type, value="ATTITUDE", bg="#161b28", fg="#00e5ff", selectcolor="#0b0f19", command=self.ai_update_labels).pack(side=tk.LEFT)
        tk.Radiobutton(type_f, text="位置环", variable=self.ai_exp_type, value="POSITION", bg="#161b28", fg="#ffaa00", selectcolor="#0b0f19", command=self.ai_update_labels).pack(side=tk.LEFT)

        phys_f = tk.Frame(top_f, bg="#161b28"); phys_f.pack(side=tk.LEFT, padx=30)
        tk.Label(phys_f, text="⚙️ 台架物理参数评估:", bg="#161b28", fg="#ffaa00", font=("微软雅黑", 9, "bold")).pack(side=tk.LEFT, padx=(0,10))
        tk.Label(phys_f, text="惯量评估(J):", bg="#161b28", fg="white").pack(side=tk.LEFT)
        self.ent_inertia = tk.Entry(phys_f, width=6, bg="#1c253c", fg="white", bd=0, justify="center"); self.ent_inertia.insert(0, "0.15"); self.ent_inertia.pack(side=tk.LEFT, padx=2)
        tk.Button(top_f, text="💾 导出 参数(JSON)", font=("微软雅黑", 9), bg="#1c253c", fg="white", command=self.ai_export_params).pack(side=tk.RIGHT, padx=15)

        param_f = tk.Frame(page, bg="#161b28", pady=10); param_f.pack(fill=tk.X, padx=10)
        head_f = tk.Frame(param_f, bg="#161b28"); head_f.pack(fill=tk.X, padx=10, pady=(5,0))
        tk.Label(head_f, text="控制通道", width=12, bg="#161b28", fg="#8f9bb3", font=("微软雅黑", 9, "bold")).pack(side=tk.LEFT)
        for t in ["外环 P (Angle/Pos)", "内环 P (Rate/Vel)", "内环 I", "内环 D (Filter)"]:
            tk.Label(head_f, text=t, width=15, bg="#161b28", fg="#8f9bb3", font=("微软雅黑", 9, "bold")).pack(side=tk.LEFT, padx=5)

        self.ai_p1_lbls = self._create_ai_param_row(param_f, "Roll 横滚", "#00e5ff", 'p1')
        self.ai_p2_lbls = self._create_ai_param_row(param_f, "Pitch 俯仰", "#ffaa00", 'p2')
        self.ai_p3_lbls = self._create_ai_param_row(param_f, "Yaw 偏航", "#a64aff", 'p3')

        tk.Button(param_f, text="⚡ 运行PID自整定", font=("微软雅黑", 10, "bold"), bg="#00e676", fg="black", padx=15, pady=4, command=self.ai_run_optimization).pack(side=tk.RIGHT, padx=20, pady=5)

        bot_f = tk.Frame(page, bg=Theme.BG_APP); bot_f.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        chart_f = tk.Frame(bot_f, bg=Theme.BG_APP); chart_f.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        self.fig_ai, (self.ax_ai_resp, self.ax_ai_err) = plt.subplots(2, 1, figsize=(6, 4.5), facecolor='#161b28', gridspec_kw={'height_ratios': [2.5, 1]})
        self.fig_ai.subplots_adjust(hspace=0.25, bottom=0.1, top=0.92, left=0.08, right=0.96)
        for ax in [self.ax_ai_resp, self.ax_ai_err]:
            ax.set_facecolor('#0b0f19'); ax.tick_params(colors='#8f9bb3', labelsize=8); ax.grid(color='#222b45', linestyle='--', linewidth=0.5)
            for spine in ax.spines.values(): spine.set_color('#222b45')
        
        self.ax_ai_resp.set_title("三通道阶跃跟踪与阻尼比分析", color="white", fontsize=10)
        self.ax_ai_err.set_title("稳态误差 (Error) 跟踪", color="white", fontsize=10)
        self.canvas_ai = FigureCanvasTkAgg(self.fig_ai, master=chart_f); self.canvas_ai.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        report_f = tk.Frame(bot_f, bg=Theme.BG_APP, width=400); report_f.pack(side=tk.RIGHT, fill=tk.Y, padx=(10,0)); report_f.pack_propagate(False)
        self.txt_ai_report = tk.Text(report_f, bg="#0d1424", fg="#00e5ff", bd=0, font=("Consolas", 9), padx=10, pady=10); self.txt_ai_report.pack(fill=tk.BOTH, expand=True)
        self.txt_ai_report.insert("1.0", ">> AI 系统就绪。\n已开启三轴全自动串级瓶颈侦测。\n请导入数据。")
        self.ai_update_labels()

    def _create_ai_param_row(self, parent, label_text, color, key):
        row_f = tk.Frame(parent, bg="#161b28"); row_f.pack(fill=tk.X, padx=10, pady=5)
        tk.Label(row_f, text=label_text, width=10, bg="#0d1424", fg=color, font=("Consolas", 10, "bold")).pack(side=tk.LEFT, padx=(0,10))
        lbls = []
        for _ in range(4):
            e = tk.Entry(row_f, width=12, bg="#1c253c", fg="white", bd=0, justify="center", font=("Consolas", 11)); e.pack(side=tk.LEFT, padx=12); self.ai_entries[key].append(e); lbls.append(e)
        return lbls

    def ai_update_labels(self):
        mode = self.ai_exp_type.get()
        self.ai_p1_names = ["Roll_Ang_P", "Roll_Rate_P", "Roll_Rate_I", "Roll_Rate_D"]
        self.ai_p2_names = ["Pitch_Ang_P", "Pitch_Rate_P", "Pitch_Rate_I", "Pitch_Rate_D"]
        self.ai_p3_names = ["Yaw_Ang_P", "Yaw_Rate_P", "Yaw_Rate_I", "Yaw_Rate_D"]
        
        self.ai_p1_lbls[0].master.winfo_children()[0].config(text="Roll 横滚" if mode == "ATTITUDE" else "X轴 前后")
        self.ai_p2_lbls[0].master.winfo_children()[0].config(text="Pitch 俯仰" if mode == "ATTITUDE" else "Y轴 左右")
        self.ai_p3_lbls[0].master.winfo_children()[0].config(text="Yaw 偏航" if mode == "ATTITUDE" else "Z轴 高度")
        
        if mode == "POSITION":
            self.ai_p1_names = ["X_Pos_P", "X_Vel_P", "X_Vel_I", "X_Vel_D"]
            self.ai_p2_names = ["Y_Pos_P", "Y_Vel_P", "Y_Vel_I", "Y_Vel_D"]
            self.ai_p3_names = ["Z_Pos_P", "Z_Vel_P", "Z_Vel_I", "Z_Vel_D"]

    def ai_save_state(self):
        try:
            state = {'p1': [float(e.get()) for e in self.ai_entries['p1']], 'p2': [float(e.get()) for e in self.ai_entries['p2']], 'p3': [float(e.get()) for e in self.ai_entries['p3']]}
            self.ai_history_params.append(state)
            if len(self.ai_history_params) > 10: self.ai_history_params.pop(0)
        except: pass

    def ai_rollback_params(self):
        if not self.ai_history_params: return messagebox.showinfo("提示", "没有可回滚的历史记录！")
        state = self.ai_history_params.pop()
        for key in ['p1', 'p2', 'p3']:
            for i, val in enumerate(state[key]): self.ai_entries[key][i].delete(0, tk.END); self.ai_entries[key][i].insert(0, str(val))
        self.txt_ai_report.insert(tk.END, "\n[系统] 已回滚至上一次参数状态。")

    def ai_export_params(self):
        try:
            data = {"Channel_1": dict(zip(self.ai_p1_names, [float(e.get()) for e in self.ai_entries['p1']])), "Channel_2": dict(zip(self.ai_p2_names, [float(e.get()) for e in self.ai_entries['p2']])), "Channel_3": dict(zip(self.ai_p3_names, [float(e.get()) for e in self.ai_entries['p3']]))}
            fn = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
            if fn:
                with open(fn, 'w') as f: json.dump(data, f, indent=4)
                messagebox.showinfo("成功", "三轴参数导出成功！")
        except Exception as e: messagebox.showerror("错误", str(e))

    def ai_load_csv(self):
        filepath = filedialog.askopenfilename(filetypes=[("CSV Files", "*.csv")])
        if not filepath: return
        try:
            df_raw = pd.read_csv(filepath); df_raw = df_raw.dropna(subset=['time', 'tgt1', 'act1', 'tgt2', 'act2'])
            self.ai_df = df_raw.sort_values('time').reset_index(drop=True)
            dts = np.diff(self.ai_df['time'].values); valid_dts = dts[dts > 1e-6] 
            if len(valid_dts) > 0: self.ai_fs = np.clip(1.0 / np.median(valid_dts), 5.0, 1000.0) 
            else: self.ai_fs = 50.0

            self.ai_save_state()
            for axis, ui_key in [('p1', 'p1'), ('p2', 'p2'), ('p3', 'p3')]:
                if f'out_{axis}' in self.ai_df.columns:
                    for idx, key in enumerate([f'out_{axis}', f'in_{axis}', f'in_i{axis[-1]}', f'in_d{axis[-1]}']):
                        self.ai_entries[ui_key][idx].delete(0, tk.END); self.ai_entries[ui_key][idx].insert(0, str(self.ai_df[key].iloc[0]))
        
            self.ax_ai_resp.clear(); self.ax_ai_err.clear()
            t = self.ai_df['time'].values - self.ai_df['time'].values[0] 
            
            if np.max(np.abs(self.ai_df['tgt1'])) > 1e-3:
                self.ax_ai_resp.plot(t, self.ai_df['tgt1'], color="#555", linestyle='--', label="Tgt 1")
                self.ax_ai_resp.plot(t, self.ai_df['act1'], color="#00e5ff", label="Act 1 (Roll/X)")
                self.ax_ai_err.plot(t, self.ai_df['tgt1'] - self.ai_df['act1'], color="#00e5ff", alpha=0.7)
            if np.max(np.abs(self.ai_df['tgt2'])) > 1e-3:
                self.ax_ai_resp.plot(t, self.ai_df['tgt2'], color="#775533", linestyle='--', label="Tgt 2")
                self.ax_ai_resp.plot(t, self.ai_df['act2'], color="#ffaa00", label="Act 2 (Pitch/Y)")
                self.ax_ai_err.plot(t, self.ai_df['tgt2'] - self.ai_df['act2'], color="#ffaa00", alpha=0.7)
            if 'tgt3' in self.ai_df.columns and np.max(np.abs(self.ai_df['tgt3'])) > 1e-3:
                self.ax_ai_resp.plot(t, self.ai_df['tgt3'], color="#6644aa", linestyle='--', label="Tgt 3")
                self.ax_ai_resp.plot(t, self.ai_df['act3'], color="#a64aff", label="Act 3 (Yaw/Z)")
                self.ax_ai_err.plot(t, self.ai_df['tgt3'] - self.ai_df['act3'], color="#a64aff", alpha=0.7)

            self.ax_ai_resp.legend(facecolor='#161b28', labelcolor='white', loc='upper right', fontsize=8); self.ax_ai_err.axhline(0, color='white', linestyle=':', linewidth=0.8)
            self.canvas_ai.draw(); self.txt_ai_report.delete("1.0", tk.END); self.txt_ai_report.insert("1.0", f"✅ 数据清洗完成\n样本数: {len(self.ai_df)}\n真实采样率: {self.ai_fs:.1f} Hz\n请点击【运行整定】。")
        except Exception as e: messagebox.showerror("解析失败", f"数据异常:\n{str(e)}")

    def _ai_extract_metrics(self, t_seq, target_amp, raw_act):
        window = max(3, int(self.ai_fs * 0.05)); act_filt = pd.Series(raw_act).rolling(window=window, center=True, min_periods=1).mean().values
        tail_len = max(5, int(len(act_filt) * 0.2)); steady_val = np.mean(act_filt[-tail_len:])
        err_ratio = abs(target_amp - steady_val) / (abs(target_amp) + 1e-5)
        true_max = np.max(act_filt) if target_amp > 0 else np.min(act_filt); safe_steady = steady_val if abs(steady_val) > 1e-3 else target_amp
        overshoot = max(0.0, (abs(true_max) - abs(safe_steady)) / abs(safe_steady))
        t_s = 999.0 
        for i in range(len(act_filt)-1, -1, -1):
            if abs(act_filt[i] - safe_steady) > 0.05 * abs(safe_steady):
                if i + 1 < len(t_seq): t_s = t_seq[i+1] - t_seq[0]
                break
        if t_s == 999.0 and len(t_seq) > 0: t_s = t_seq[-1] - t_seq[0]
        dt = 1.0 / self.ai_fs; vel = np.abs(np.diff(raw_act)) / dt; mean_vel = np.mean(vel[int(len(vel)*0.5):]) if len(vel) > 10 else 0; jitter = np.std(raw_act - act_filt)
        return { 'steady_val': steady_val, 'err_ratio': err_ratio, 'overshoot': overshoot, 't_s': t_s, 'mean_vel': mean_vel, 'jitter': jitter, 'max_resp': np.max(np.abs(raw_act)) }

    def _ai_tune_single_axis(self, target_amp, target_arr, actual, time_arr, p_arr):
        out_p, in_p, in_i, in_d = p_arr; reasoning = []
        try: inertia_J = float(self.ent_inertia.get())
        except: inertia_J = 0.15
        J_ratio = max(0.5, min(inertia_J / 0.15, 2.0)); MAX_OUT_P = 5.0 / J_ratio; MAX_IN_P = 0.2 * J_ratio; MAX_IN_I = 0.3; MAX_IN_D = 0.005 * J_ratio
        if in_d < 1e-4: in_d = 0.001
        if in_i < 1e-4: in_i = 0.01
        valid_idx = np.where(np.abs(target_arr) >= np.abs(target_amp) * 0.5)[0]
        if len(valid_idx) < int(self.ai_fs * 0.5): return p_arr, ["❌ 数据截取失败：阶跃保持时间短于 0.5 秒！"]
        m = self._ai_extract_metrics(time_arr[valid_idx], target_amp, actual[valid_idx]); cost = 0.4 * m['err_ratio'] + 0.4 * m['overshoot'] + 0.2 * (m['t_s'] / 2.0)
        reasoning.append(f"📊 指标评估:\n超调={m['overshoot']*100:.1f}%\n静差={m['err_ratio']*100:.1f}%\nT_s={m['t_s']:.2f}s")
        vel_norm = m['mean_vel'] / (abs(target_amp) + 1e-5)
        if m['max_resp'] < abs(target_amp) * 0.05 or (m['err_ratio'] > 0.3 and vel_norm < 0.1): reasoning.append("⚫ 【致命死锁拦截】响应极弱或卡滞。"); return p_arr, reasoning
        n_out_p, n_in_p, n_in_i, n_in_d = out_p, in_p, in_i, in_d
        if m['jitter'] > 0.05 * abs(target_amp) or (n_out_p > 4.0 and m['overshoot'] > 0.1): reasoning.append("🔹 [🔴 内环高频发散]"); reasoning.append("➤ 首砍内环 D, 降外环。"); n_in_p *= 0.8; n_in_d *= 0.5; n_out_p *= 0.8
        elif m['overshoot'] > 0.10:
            if n_out_p > 2.5: reasoning.append("🔹 [🔸 外环逼迫过猛]"); reasoning.append("➤ 降低外环放缓节奏。"); n_out_p *= 0.75
            else: reasoning.append("🔹 [🔴 内环阻尼严重匮乏]"); reasoning.append("➤ 强加内环 D 项。"); n_in_d = min(n_in_d * 1.2 + 0.0005, MAX_IN_D); n_in_p *= 0.9
        elif m['err_ratio'] > 0.08:
            reasoning.append("🔹 [🟡 稳态静差阻滞]")
            if n_in_i > MAX_IN_I * 0.8: reasoning.append("⚠️ 积分已达上限。")
            else: reasoning.append("➤ 增加内环积分 I。"); n_in_i = min(n_in_i * (1.0 + m['err_ratio'] * 2.0), MAX_IN_I)
        elif m['t_s'] > 0.8: reasoning.append("🔹 [⚪ 系统响应拖沓]"); reasoning.append("➤ 提升内外环 P 以提速。"); n_out_p = min(n_out_p * 1.15, MAX_OUT_P); n_in_p = min(n_in_p * 1.1, MAX_IN_P)
        else: reasoning.append("🔹 [🟢 已达黄金容差带]")
        return [np.clip(n_out_p, 0, MAX_OUT_P), np.clip(n_in_p, 0, MAX_IN_P), np.clip(n_in_i, 0, MAX_IN_I), np.clip(n_in_d, 0, MAX_IN_D)], reasoning

    def ai_run_optimization(self):
        if self.ai_df is None: return messagebox.showwarning("提示", "请先导入数据！")
        self.ai_save_state()
        try: p1 = [float(e.get()) for e in self.ai_entries['p1']]; p2 = [float(e.get()) for e in self.ai_entries['p2']]; p3 = [float(e.get()) for e in self.ai_entries['p3']]
        except: return messagebox.showerror("参数非法", "参数框内必须为纯数字！")
        t_arr = self.ai_df['time'].values; report = f"🛡️ V4.2 三轴量化辨识报告\n" + "="*30 + "\n"; new_p = {'p1': p1.copy(), 'p2': p2.copy(), 'p3': p3.copy()}
        tasks = [('p1', 'tgt1', 'act1', p1, self.ai_p1_names, "横滚/X轴"), ('p2', 'tgt2', 'act2', p2, self.ai_p2_names, "俯仰/Y轴")]
        if 'tgt3' in self.ai_df.columns: tasks.append(('p3', 'tgt3', 'act3', p3, self.ai_p3_names, "偏航/Z轴"))
        for axis, t_col, a_col, curr_p, names, title in tasks:
            df_t = self.ai_df[t_col].values; df_a = self.ai_df[a_col].values; max_tgt = np.max(np.abs(df_t))
            if max_tgt > 1e-3:
                np_arr, r_log = self._ai_tune_single_axis(max_tgt, df_t, df_a, t_arr, curr_p); new_p[axis] = np_arr
                report += f"【{title} 诊断】\n" + "\n".join([" " + r for r in r_log]) + "\n"
                for i, val in enumerate(np_arr): chg = " ~" if abs(val - curr_p[i]) < 1e-4 else f" ⟾ {val:.4f}"; report += f"[{names[i][:8]}] : {curr_p[i]:.4f}{chg}\n"
                report += "-"*30 + "\n"
        self.txt_ai_report.delete("1.0", tk.END); self.txt_ai_report.insert("1.0", report)
        for key in ['p1', 'p2', 'p3']:
            for idx, e in enumerate(self.ai_entries[key]): e.delete(0, tk.END); e.insert(0, f"{new_p[key][idx]:.4f}")


    def switch_tune_mode(self):
        mode = self.loop_mode_var.get(); state.current_control_mode = mode 
        self.rb_axis1.config(text="Roll" if mode=="ATTITUDE" else "X (前/后)", value="Roll" if mode=="ATTITUDE" else "Pos_X")
        self.rb_axis2.config(text="Pitch" if mode=="ATTITUDE" else "Y (左/右)", value="Pitch" if mode=="ATTITUDE" else "Pos_Y")
        self.rb_axis3.config(text="Yaw" if mode=="ATTITUDE" else "Z (高度)", value="Yaw" if mode=="ATTITUDE" else "Pos_Z")
        self.rb_axis4.config(text="All(齐发)", value="All"); self.tune_axis_var.set("Roll" if mode=="ATTITUDE" else "Pos_X")
        self.ent_step_amp.delete(0, tk.END); self.ent_step_amp.insert(0, "15" if mode == "ATTITUDE" else "0.15") 
        for widget in self.param_container.winfo_children(): widget.destroy()
        self.manual_entries.clear()
        if mode == "ATTITUDE": configs = [("Roll 横滚", Theme.COLOR_ROLL, "Att_Roll", ["KP_ROLL_ANGLE", "KP_ROLL_RATE", "KI_ROLL_RATE", "KD_ROLL_RATE"]), ("Pitch 俯仰", Theme.COLOR_PITCH, "Att_Pitch", ["KP_PITCH_ANGLE", "KP_PITCH_RATE", "KI_PITCH_RATE", "KD_PITCH_RATE"]), ("Yaw 偏航", Theme.COLOR_YAW, "Att_Yaw", ["KP_YAW_ANGLE", "KP_YAW_RATE", "KI_YAW_RATE", "KD_YAW_RATE"])]
        else: configs = [("X轴 前后", Theme.COLOR_ROLL, "Pos_X", ["KPXP", "KVXP", "KVXI", "KVXD"]), ("Y轴 左右", Theme.COLOR_PITCH, "Pos_Y", ["KPYP", "KVYP", "KVYI", "KVYD"]), ("Z轴 高度", Theme.COLOR_YAW, "Pos_Z", ["KPZP", "KVZP", "KVZI", "KVZD"])]
        for ax_name, color, prefix, lbls in configs:
            row = tk.Frame(self.param_container, bg=Theme.BG_CARD); row.pack(fill=tk.X, pady=5); tk.Label(row, text=ax_name, bg=Theme.BG_BOX, fg=color, font=("Consolas", 9, "bold"), width=10).pack(side=tk.LEFT, padx=10)
            for i, p_suf in enumerate(["Out_P", "In_P", "In_I", "In_D"]):
                cell = tk.Frame(row, bg=Theme.BG_CARD); cell.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5); tk.Label(cell, text=lbls[i], bg=Theme.BG_CARD, fg="#00e5ff" if i==0 else "#ffaa00", font=("Consolas", 10, "bold")).pack()
                try: ent = tk.Spinbox(cell, from_=0.0, to=30.0, increment=0.005 if "D" in p_suf else 0.05, format="%.4f", bg=Theme.BG_APP, fg=Theme.TEXT_MAIN, bd=0, justify='center', font=("Consolas", 10, "bold"), buttonbackground=Theme.BG_INPUT)
                except: ent = tk.Spinbox(cell, from_=0.0, to=30.0, format="%.4f", bg=Theme.BG_APP, fg=Theme.TEXT_MAIN, bd=0, justify='center', font=("Consolas", 10, "bold"), buttonbackground=Theme.BG_INPUT)
                ent.delete(0, tk.END); ent.insert(0, "0.0000"); ent.pack(fill=tk.X, ipady=4, pady=2); self.manual_entries[f"{prefix}_{p_suf}"] = ent
        self.root.after(100, self.refresh_pids)

    def write_manual_params(self):
        self.root.focus_set(); 
        if not self.comm: return messagebox.showwarning("提示", "未连接飞控！")
        fallback_map = {"Att_Roll_Out_P": "KP_ROLL_ANGLE", "Att_Roll_In_P": "KP_ROLL_RATE", "Att_Roll_In_I": "KI_ROLL_RATE", "Att_Roll_In_D": "KD_ROLL_RATE", "Att_Pitch_Out_P": "KP_PITCH_ANGLE", "Att_Pitch_In_P": "KP_PITCH_RATE", "Att_Pitch_In_I": "KI_PITCH_RATE", "Att_Pitch_In_D": "KD_PITCH_RATE", "Att_Yaw_Out_P": "KP_YAW_ANGLE", "Att_Yaw_In_P": "KP_YAW_RATE", "Att_Yaw_In_I": "KI_YAW_RATE", "Att_Yaw_In_D": "KD_YAW_RATE", "Pos_X_Out_P": "KPXP", "Pos_X_In_P": "KVXP", "Pos_X_In_I": "KVXI", "Pos_X_In_D": "KVXD", "Pos_Y_Out_P": "KPYP", "Pos_Y_In_P": "KVYP", "Pos_Y_In_I": "KVYI", "Pos_Y_In_D": "KVYD", "Pos_Z_Out_P": "KPZP", "Pos_Z_In_P": "KVZP", "Pos_Z_In_I": "KVZI", "Pos_Z_In_D": "KVZD"}
        count = 0
        for ui_key, ent in self.manual_entries.items():
            try:
                val = float(ent.get()); limit = SafetyConfig.MAX_PID['P']
                if "In_I" in ui_key: limit = SafetyConfig.MAX_PID['I']
                if "In_D" in ui_key: limit = SafetyConfig.MAX_PID['D']
                if val < 0 or val > limit: messagebox.showerror("安全拦截", f"参数 {ui_key} 的值 ({val}) 超过安全范围 (0 ~ {limit})，已拒绝写入！"); return
                real_param_name = state.real_param_map.get(ui_key)
                if not real_param_name: real_param_name = fallback_map.get(ui_key)
                if real_param_name: self.comm.set_param_async(real_param_name, val); count += 1
            except ValueError: pass
        state.add_log(f">>> 成功向固件下发 {count} 个控制参数！"); self.root.after(500, self.refresh_pids)

    def refresh_pids(self):
        if self.comm: self.comm.request_pid_params()

    def toggle_send_button(self):
        if not self.comm: return messagebox.showwarning("提示", "请先连接飞控！")
        self.is_send_enabled = not getattr(self, 'is_send_enabled', False)
        if self.is_send_enabled:
            state.send_enabled = True; self.btn_send.config(text="[1] 停止发送指令", bg="#4a4a4a", fg="white"); self.btn_arm.config(state="normal", bg=Theme.COLOR_THRUST, fg="#000") ; state.add_log("指令流已启动，原点对齐完毕，允许进行解锁")
        else:
            state.send_enabled = False; self.btn_send.config(text="[1] 开启指令发送", bg=Theme.COLOR_THRUST, fg="#000")
            if getattr(self, 'is_arming_requested', False): self.toggle_arm_button()
            self.btn_arm.config(state="disabled", bg=Theme.BTN_DISABLED, fg="#888"); state.add_log("指令流已停止"); state.is_step_testing = False

    def toggle_arm_button(self):
        if not self.comm: return
        self.is_arming_requested = not getattr(self, 'is_arming_requested', False)
        if self.is_arming_requested: self.comm.handle_arm_switch(True); self.btn_arm.config(text="[2] 🚨 紧急上锁 (DISARM)", bg=Theme.COLOR_DANGER, fg=Theme.TEXT_MAIN)
        else: self.comm.handle_arm_switch(False); self.btn_arm.config(text="[2] 允许解锁电机 (ARM)", bg=Theme.COLOR_THRUST, fg="#000"); state.is_step_testing = False

    def run_step_test_thread(self):
        if not self.comm or getattr(state, 'is_step_testing', False): return
        if not getattr(self, 'is_send_enabled', False) or not getattr(self, 'is_arming_requested', False): return messagebox.showerror("安全防线", "请先开启[1]指令发送，并[2]解锁电机。")
        threading.Thread(target=self._step_test_task, daemon=True).start()

    def _step_test_task(self):
        state.is_step_testing = True; self.btn_step_test.config(text="测试执行中(请勿干预)...", bg=Theme.BG_BOX, fg=Theme.TEXT_SUB, state=tk.DISABLED)
        axis_val = self.tune_axis_var.get(); mode = self.loop_mode_var.get()
        try: amp = float(self.ent_step_amp.get())
        except ValueError: self.abort_test("幅度输入无效，请确保输入为纯数字！"); return
        if mode == "ATTITUDE":
            if abs(amp) > SafetyConfig.MAX_ATTITUDE_ANGLE: self.abort_test(f"安全拦截：姿态阶跃幅度({amp}°)超限！\n允许最大值为 {SafetyConfig.MAX_ATTITUDE_ANGLE}°。"); return
        else:
            if axis_val in ("Pos_X", "Pos_Y", "All") and abs(amp) > SafetyConfig.MAX_XY_TRAVEL: self.abort_test(f"安全拦截：平移幅度({amp}m)超过台架极限({SafetyConfig.MAX_XY_TRAVEL}m)！"); return
            if axis_val in ("Pos_Z", "All") and abs(amp) > SafetyConfig.MAX_Z_TRAVEL: self.abort_test(f"安全拦截：高度幅度({amp}m)超过台架极限({SafetyConfig.MAX_Z_TRAVEL}m)！"); return
        
        base_r = state.target_roll if mode == "ATTITUDE" else state.roll
        base_p = state.target_pitch if mode == "ATTITUDE" else state.pitch
        base_y_ang = state.target_yaw if mode == "ATTITUDE" else state.yaw
        base_x = state.pos_x; base_y = state.pos_y; base_z = state.pos_z 

        state.add_log(f">>> 开始执行纯正的【相对增量瞬间阶跃】测试 ({axis_val})")
        with state._lock:
            state.test_time_buf.clear(); state.test_tgt1_buf.clear(); state.test_act1_buf.clear(); state.test_tgt2_buf.clear(); state.test_act2_buf.clear(); state.test_tgt3_buf.clear(); state.test_act3_buf.clear()

        def step_action(step_val):
            if mode == "ATTITUDE":
                state.target_roll = base_r + (step_val if axis_val in ("Roll", "All") else 0)
                state.target_pitch = base_p + (step_val if axis_val in ("Pitch", "All") else 0)
                state.target_yaw = base_y_ang + (step_val if axis_val in ("Yaw", "All") else 0)
            else:
                x_cmd = step_val if axis_val in ("Pos_X", "All") else 0
                y_cmd = step_val if axis_val in ("Pos_Y", "All") else 0
                z_cmd_diff = step_val if axis_val in ("Pos_Z", "All") else 0
                state.test_target_x = np.clip(base_x + x_cmd, base_x - SafetyConfig.MAX_XY_TRAVEL, base_x + SafetyConfig.MAX_XY_TRAVEL)
                state.test_target_y = np.clip(base_y + y_cmd, base_y - SafetyConfig.MAX_XY_TRAVEL, base_y + SafetyConfig.MAX_XY_TRAVEL)
                state.test_target_z = np.clip(base_z - z_cmd_diff, base_z - SafetyConfig.MAX_Z_TRAVEL, base_z + 0.1)

            with state._lock:
                state.test_time_buf.append(time.time() - start_time)
                if mode == "ATTITUDE":
                    state.test_tgt1_buf.append(step_val if axis_val in ("Roll", "All") else 0); state.test_act1_buf.append(state.roll - base_r)
                    state.test_tgt2_buf.append(step_val if axis_val in ("Pitch", "All") else 0); state.test_act2_buf.append(state.pitch - base_p)
                    state.test_tgt3_buf.append(step_val if axis_val in ("Yaw", "All") else 0); state.test_act3_buf.append(state.yaw - base_y_ang)
                else:
                    state.test_tgt1_buf.append(step_val if axis_val in ("Pos_X", "All") else 0); state.test_act1_buf.append(state.pos_x - base_x)
                    state.test_tgt2_buf.append(step_val if axis_val in ("Pos_Y", "All") else 0); state.test_act2_buf.append(state.pos_y - base_y)
                    state.test_tgt3_buf.append(step_val if axis_val in ("Pos_Z", "All") else 0); state.test_act3_buf.append(-(state.pos_z - base_z))

        start_time = time.time()
        while time.time() - start_time < 0.5:
            if not state.is_step_testing or not state.send_enabled or not state.connected: break 
            step_action(0); time.sleep(0.02)
        step_start = time.time()
        while time.time() - step_start < 2.0:
            if not state.is_step_testing or not state.send_enabled or not state.connected: break 
            step_action(amp); time.sleep(0.02)
        end_start = time.time()
        while time.time() - end_start < 0.5:
            if not state.is_step_testing or not state.send_enabled or not state.connected: break 
            step_action(0); time.sleep(0.02)
            
        if state.connected and state.send_enabled: step_action(0)
        was_aborted = not state.is_step_testing or not state.connected; state.is_step_testing = False
        if was_aborted: state.add_log(">>> 测试已被紧急中断！")
        else: self.root.after(0, lambda: self._calculate_and_show_metrics(amp))
        self.root.after(0, lambda: self.btn_step_test.config(text="▶ 安全运行阶跃测试", bg=Theme.COLOR_THRUST, fg="#000", state=tk.NORMAL))

    def abort_test(self, reason):
        messagebox.showerror("安全拦截", reason); state.is_step_testing = False
        self.root.after(0, lambda: self.btn_step_test.config(text="▶ 安全运行阶跃测试", bg=Theme.COLOR_THRUST, fg="#000", state=tk.NORMAL))

    def _calculate_and_show_metrics(self, target_amp):
        if not state.test_time_buf: return
        times = np.array(state.test_time_buf) - state.test_time_buf[0]; ax = self.tune_axis_var.get()
        if ax in ["Pitch", "Pos_Y"]: actuals = np.array(state.test_act2_buf); tgt_buf = state.test_tgt2_buf
        elif ax in ["Yaw", "Pos_Z"]: actuals = np.array(state.test_act3_buf); tgt_buf = state.test_tgt3_buf
        else: actuals = np.array(state.test_act1_buf); tgt_buf = state.test_tgt1_buf
        
        steady_val = np.mean(actuals[-20:]) if len(actuals) >= 20 else actuals[-1]
        peak_val = np.max(actuals); overshoot = max(0, (peak_val - steady_val) / steady_val * 100) if steady_val > 0.5 else 0.0; steady_error = abs(steady_val - target_amp)
        try: t10 = times[np.where(actuals >= 0.1 * steady_val)[0][0]]; t90 = times[np.where(actuals >= 0.9 * steady_val)[0][0]]; rise_time = (t90 - t10) * 1000
        except: rise_time = 0

        self.perf_labels["超调量"].config(text=f"{overshoot:.1f}"); self.perf_labels["上升时间"].config(text=f"{rise_time:.0f}"); self.perf_labels["稳态误差"].config(text=f"{steady_error:.2f}")

        self.line_target.set_data(state.test_time_buf, tgt_buf)
        if np.max(state.test_tgt1_buf) > 0: self.line_act1.set_data(state.test_time_buf, state.test_act1_buf)
        else: self.line_act1.set_data([], [])
        if np.max(state.test_tgt2_buf) > 0: self.line_act2.set_data(state.test_time_buf, state.test_act2_buf)
        else: self.line_act2.set_data([], [])
        if np.max(state.test_tgt3_buf) > 0: self.line_act3.set_data(state.test_time_buf, state.test_act3_buf)
        else: self.line_act3.set_data([], [])
            
        self.ax_tune.set_xlim(state.test_time_buf[0], state.test_time_buf[-1])
        all_acts = state.test_act1_buf + state.test_act2_buf + state.test_act3_buf
        self.ax_tune.set_ylim(min(all_acts)-2, max(max(all_acts), target_amp) + 5)
        self.canvas_tune.draw_idle()

    def export_test_data(self):
        if not state.test_time_buf: return messagebox.showwarning("提示", "请先运行一次阶跃测试！")
        mode = self.loop_mode_var.get(); axis = self.tune_axis_var.get()
        p1_prefix = "Att_Roll" if mode == "ATTITUDE" else "Pos_X"; p2_prefix = "Att_Pitch" if mode == "ATTITUDE" else "Pos_Y"; p3_prefix = "Att_Yaw" if mode == "ATTITUDE" else "Pos_Z"
        def get_p(prefix):
            try: return [float(self.manual_entries[f"{prefix}_{s}"].get()) for s in ["Out_P", "In_P", "In_I", "In_D"]]
            except Exception: return [0.0, 0.0, 0.0, 0.0]
        p1 = get_p(p1_prefix); p2 = get_p(p2_prefix); p3 = get_p(p3_prefix)

        fn = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=f"SuperData_{mode}_{axis}.csv", filetypes=[("CSV", "*.csv")])
        if fn:
            try:
                with open(fn, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time', 'tgt1', 'act1', 'out_p1', 'in_p1', 'in_i1', 'in_d1', 'tgt2', 'act2', 'out_p2', 'in_p2', 'in_i2', 'in_d2', 'tgt3', 'act3', 'out_p3', 'in_p3', 'in_i3', 'in_d3'])
                    for i in range(len(state.test_time_buf)):
                        writer.writerow([state.test_time_buf[i], state.test_tgt1_buf[i], state.test_act1_buf[i], p1[0], p1[1], p1[2], p1[3], state.test_tgt2_buf[i], state.test_act2_buf[i], p2[0], p2[1], p2[2], p2[3], state.test_tgt3_buf[i], state.test_act3_buf[i], p3[0], p3[1], p3[2], p3[3]])
                messagebox.showinfo("导出成功", "三轴数据阵列已打包！可直接切换到【离线 PID AI 自整定】页面导入推演。")
            except Exception as e: messagebox.showerror("错误", f"导出失败: {e}")

    def export_csv(self):
        if not state.telemetry_data: return messagebox.showinfo("提示", "当前无数据可导出！请先连接飞控。")
        fn = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=f"telemetry_{int(time.time())}.csv", filetypes=[("CSV 文件", "*.csv")])
        if fn:
            try:
                with open(fn, 'w', newline='') as f:
                    writer = csv.writer(f); writer.writerow(['time', 'roll', 'pitch', 'yaw', 'thrust', 'm1', 'm2', 'm3', 'm4'])
                    with state._lock: writer.writerows(state.telemetry_data)
                messagebox.showinfo("导出成功", f"成功导出 {len(state.telemetry_data)} 条主遥测数据！")
            except Exception as e: messagebox.showerror("导出失败", f"文件保存失败：\n{e}")

    def create_metric_card(self, parent, title, color):
        outer = tk.Frame(parent, bg=Theme.BORDER, padx=1, pady=1); outer.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)
        inner = tk.Frame(outer, bg=Theme.BG_CARD, padx=10, pady=2); inner.pack(fill=tk.BOTH, expand=True)
        tk.Label(inner, text=title, fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("微软雅黑", 9)).pack()
        lbl = tk.Label(inner, text="0.0", fg=color, bg=Theme.BG_CARD, font=("Consolas", 24, "bold")); lbl.pack(); return lbl

    def create_canvas_slider(self, p, lbl, min_v, max_v, tag, color):
        f = tk.Frame(p, bg=Theme.BG_CARD); f.pack(fill=tk.X, pady=6)
        h = tk.Frame(f, bg=Theme.BG_CARD); h.pack(fill=tk.X)
        tk.Label(h, text=lbl, bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("Consolas", 10)).pack(side=tk.LEFT)
        v_lbl = tk.Label(h, text="0.0", bg=Theme.BG_CARD, fg=color, font=("Consolas", 10, "bold")); v_lbl.pack(side=tk.RIGHT)
        cv = tk.Canvas(f, height=24, bg=Theme.BG_CARD, highlightthickness=0, cursor="hand2"); cv.pack(fill=tk.X, pady=(4, 2))
        
        def update_logic(val):
            v_lbl.config(text=f"{val:.1f}")
            if tag == 'roll': state.target_roll = val
            elif tag == 'pitch': state.target_pitch = val
            elif tag == 'yaw': state.target_yaw = val
            elif tag == 'thrust': state.target_thrust = (val - 1000) / 1000.0
                
        def render(event=None):
            cv.delete("all"); w = cv.winfo_width()
            if w < 30: return
            cv.create_rectangle(0, 5, w, 19, fill=Theme.BG_BOX, outline=Theme.BORDER)
            curr_val = 1100.0 if tag == 'thrust' else getattr(state, f"target_{tag}")
            if tag == 'thrust' and not getattr(self, 'is_arming_requested', False): curr_val = 1100.0 
            if event:
                raw_x = max(0, min(event.x - 12, w - 24)); val = (raw_x / (w - 24)) * (max_v - min_v) + min_v
                update_logic(val); pos_x = raw_x
            else: pos_x = ((curr_val - min_v) / (max_v - min_v)) * (w - 24)
            cv.create_rectangle(pos_x, 2, pos_x + 24, 22, fill="#4a4a4a", outline="#222")
            
        cv.bind("<B1-Motion>", render); cv.bind("<Button-1>", render); cv.bind("<Configure>", lambda e: f.after(10, render))
        return cv

    def setup_2x2_charts(self, parent):
        self.fig_status = Figure(figsize=(10, 2), facecolor=Theme.BG_CARD); self.fig_status.subplots_adjust(left=0.04, right=0.98, top=0.85, bottom=0.15, wspace=0.12, hspace=0.45)
        self.ax_status = []; self.line_status = []; self.x_data = np.arange(100); self.y_data = [np.zeros(100) for _ in range(4)]
        configs = [("ROLL 滚转", Theme.COLOR_ROLL, -40, 40, [-40, -20, 0, 20, 40]), ("PITCH 俯仰", Theme.COLOR_PITCH, -40, 40, [-40, -20, 0, 20, 40]), ("YAW 偏航", Theme.COLOR_YAW, -180, 180, [-180, -90, 0, 90, 180]), ("THRUST 推力", Theme.COLOR_THRUST, 0, 1.0, [0, 0.2, 0.4, 0.6, 0.8, 1.0])]
        for i, (title, color, ymin, ymax, yticks) in enumerate(configs):
            ax = self.fig_status.add_subplot(2, 2, i+1, facecolor=Theme.BG_APP); ax.set_title(title, color=Theme.TEXT_MAIN, fontsize=9, loc='left'); ax.tick_params(colors=Theme.TEXT_SUB, labelsize=7)
            for spine in ax.spines.values(): spine.set_color(Theme.BORDER)
            ax.grid(True, color=Theme.BORDER, linestyle='-', linewidth=0.5); ax.set_ylim(ymin, ymax); ax.set_yticks(yticks); l, = ax.plot(self.x_data, self.y_data[i], color=color, lw=1.5); self.ax_status.append(ax); self.line_status.append(l)
        self.canvas_status = FigureCanvasTkAgg(self.fig_status, master=parent); self.canvas_status.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def draw_attitude(self, cv, roll, pitch):
        cv.delete("all")
        w = cv.winfo_width(); h = cv.winfo_height()
        if w < 50 or h < 50: return
        cx, cy = w / 2, h / 2; R = min(w, h) / 2 - 25 ; cv.create_rectangle(0, 0, w, h, fill=Theme.BG_CARD, outline="")
        pitch_px = max(min(pitch * 2.0, R), -R); angle = math.radians(-roll); L = R * 3; y_center = cy + pitch_px
        dx = L * math.cos(angle); dy = L * math.sin(angle); x1 = cx - dx; y1 = y_center + dy; x2 = cx + dx; y2 = y_center - dy
        cv.create_polygon(x1, y1, x2, y2, cx+L, cy-L, cx-L, cy-L, fill="#1e5f8f", outline=""); cv.create_polygon(x1, y1, x2, y2, cx+L, cy+L, cx-L, cy+L, fill="#6e421c", outline="") 
        cv.create_line(x1, y1, x2, y2, fill="white", width=2); mask_w = max(w, h); cv.create_oval(cx-R-mask_w, cy-R-mask_w, cx+R+mask_w, cy+R+mask_w, outline=Theme.BG_CARD, width=mask_w*2); cv.create_oval(cx-R, cy-R, cx+R, cy+R, outline=Theme.COLOR_ROLL, width=2) 
        arc_R = R + 12; cv.create_arc(cx-arc_R, cy-arc_R, cx+arc_R, cy+arc_R, start=30, extent=120, style=tk.ARC, outline="white", width=1)
        ptr_rad = math.radians(roll - 90); px1 = cx + (arc_R-2) * math.cos(ptr_rad); py1 = cy + (arc_R-2) * math.sin(ptr_rad); px2 = cx + (arc_R-10) * math.cos(ptr_rad-0.1); py2 = cy + (arc_R-10) * math.sin(ptr_rad-0.1); px3 = cx + (arc_R-10) * math.cos(ptr_rad+0.1); py3 = cy + (arc_R-10) * math.sin(ptr_rad+0.1)
        cv.create_polygon(px1, py1, px2, py2, px3, py3, fill=Theme.COLOR_ROLL, outline="")

    def toggle_conn(self):
        if not self.comm:
            try:
                self.comm = MavlinkThread(self.ent_local.get(), self.ent_ip.get(), self.ent_port.get()); self.comm.start(); self.btn_conn.config(text="断开连接", bg=Theme.COLOR_DANGER, fg=Theme.TEXT_MAIN); self.btn_send.config(state="normal", bg=Theme.COLOR_THRUST, fg="#000")
            except Exception as e: state.add_log(f"错误: {e}")
        else:
            self.comm.stop(); self.comm.join(); self.comm = None; state.connected = False; self.btn_conn.config(text="启 动 连 接", bg=Theme.COLOR_DANGER, fg=Theme.TEXT_MAIN) ; self.btn_send.config(state="disabled", text="[1] 开启指令发送", bg=Theme.BTN_DISABLED, fg="#888"); self.btn_arm.config(state="disabled", text="[2] 允许解锁电机 (ARM)", bg=Theme.BTN_DISABLED, fg="#888"); self.is_send_enabled = False; self.is_arming_requested = False; state.send_enabled = False

    def update_loop(self):
        td = time.time() - state.last_packet_time
        if state.connected and td < 3.0:
            self.cv_dot.itemconfig(self.dot_id, fill=Theme.COLOR_THRUST)
            if hasattr(self, 'lbl_big_link'): self.lbl_big_link.config(text="连接正常 (LINK OK)", fg=Theme.COLOR_THRUST)
            if hasattr(self, 'lbl_latency_top'): self.lbl_latency_top.config(text=f"{td*1000:.1f} ms")
            if hasattr(self, 'lbl_lat_bot'): self.lbl_lat_bot.config(text=f"{td*1000:.1f} ms")
            if hasattr(self, 'lbl_sysid'): self.lbl_sysid.config(text=f"{state.target_system}")
            if hasattr(self, 'lbl_bat'): self.lbl_bat.config(text=f"{state.battery_voltage:.2f} V")
            if hasattr(self, 'lbl_v_roll'): self.lbl_v_roll.config(text=f"{state.roll:.1f}")
            if hasattr(self, 'lbl_v_pitch'): self.lbl_v_pitch.config(text=f"{state.pitch:.1f}")
            if hasattr(self, 'lbl_v_yaw'): self.lbl_v_yaw.config(text=f"{state.yaw:.1f}")
            
            with state._lock:
                row = [f"{time.time():.4f}", round(state.roll, 1), round(state.pitch, 1), round(state.yaw, 1), round(state.target_thrust, 3), state.servo_outputs[0], state.servo_outputs[1], state.servo_outputs[2], state.servo_outputs[3]]
                state.telemetry_data.append(row)
                if len(state.telemetry_data) > SafetyConfig.MAX_TELEMETRY_ROWS: state.telemetry_data.pop(0)

            for i, v in enumerate([state.roll, state.pitch, state.yaw, state.target_thrust]):
                self.y_data[i] = np.roll(self.y_data[i], -1); self.y_data[i][-1] = v; self.line_status[i].set_ydata(self.y_data[i])
                
            self.plot_counter += 1
            if self.plot_counter >= 4: self.canvas_status.draw_idle(); self.plot_counter = 0
            if self.active_tab == 'console': self.draw_attitude(self.cv_att, state.roll, state.pitch)

            for i, (cv, rect_id, lbl) in enumerate(self.pwm_bars):
                val = state.servo_outputs[i]; w = cv.winfo_width()
                if w > 1: fill_w = max(0, min(w, (val - 1000) / 1000.0 * w)); cv.coords(rect_id, 0, 0, fill_w, 12) 
                lbl.config(text=str(val))

            with state._lock:
                for ui_key, v in state.param_updates.copy().items():
                    if hasattr(self, 'manual_entries') and ui_key in self.manual_entries:
                        ent = self.manual_entries[ui_key]
                        if self.root.focus_get() != ent: ent.delete(0, tk.END); ent.insert(0, f"{v:.4f}") 
                state.param_updates.clear()
        else:
            self.cv_dot.itemconfig(self.dot_id, fill=Theme.COLOR_DANGER)
            if hasattr(self, 'lbl_big_link'): self.lbl_big_link.config(text="离线 (OFFLINE)", fg=Theme.COLOR_DANGER)
            if hasattr(self, 'lbl_latency_top'): self.lbl_latency_top.config(text="-- ms")

        while state.log_queue: self.log_text.insert(tk.END, state.log_queue.pop(0) + "\n"); self.log_text.see(tk.END)
        lines = int(float(self.log_text.index('end-1c')))
        if lines > 300: self.log_text.delete('1.0', f"{lines - 300 + 1}.0")
        self.root.after(50, self.update_loop)

if __name__ == "__main__":
    root = tk.Tk(); app = DroneApp(root); root.mainloop()
