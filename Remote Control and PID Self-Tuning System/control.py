import sys
import time
import math
import threading
import datetime
import numpy as np
import csv
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

class Theme:
    BG_APP = "#0b0f19"; BG_CARD = "#161b28"; BG_BOX = "#0d1424"       
    BG_INPUT = "#1c253c"; TEXT_MAIN = "#ffffff"; TEXT_SUB = "#8f9bb3"     
    COLOR_ROLL = "#00e5ff"; COLOR_PITCH = "#ffaa00"; COLOR_YAW = "#a64aff"    
    COLOR_THRUST = "#00e676"; COLOR_DANGER = "#ff3d71"; BORDER = "#222b45"       

class DroneState:
    def __init__(self):
        self._lock = threading.Lock() 
        self.roll = 0.0; self.pitch = 0.0; self.yaw = 0.0
        self.servo_outputs = [0] * 8
        self.connected = False; self.last_packet_time = 0.0
        self.target_system = 1; self.target_component = 1 
        self.flight_mode = "UNKNOWN"
        self.battery_voltage = 0.0; self.is_armed = False
        self.target_roll = 0.0; self.target_pitch = 0.0; self.target_yaw = 0.0; self.target_thrust = 0.0 
        self.send_enabled = False; self.maintain_offboard = False 
        self.autotune_running = False; self.autotune_status_msg = "Ready"
        
        self.ui_params = {
            "Roll_P": 0.0, "Roll_I": 0.0, "Roll_D": 0.0,
            "Pitch_P": 0.0, "Pitch_I": 0.0, "Pitch_D": 0.0,
            "Yaw_P": 0.0, "Yaw_I": 0.0, "Yaw_D": 0.0,
        }
        self.real_param_map = {} 
        self.old_params = {} 
        self.param_updates = {}; self.log_queue = []
        
        
        self.initial_params_snapshot = {} 
        
        self.telemetry_data = []
        self.test_time_buffer = []; self.test_target_buffer = []; self.test_actual_buffer = []
        self.is_step_testing = False

    def add_log(self, msg):
        ts = datetime.datetime.now().strftime("%H:%M:%S")
        with self._lock: self.log_queue.append(f"[{ts}] {msg}")

    def update_param(self, k, v):
        with self._lock:
            ui_key = None
            # 智能特征识别：涵盖 PX4新版、PX4老版、ArduPilot
            if k in ("ATC_RAT_RLL_P", "MC_RR_P", "MC_ROLLRATE_P"): ui_key = "Roll_P"
            elif k in ("ATC_RAT_RLL_I", "MC_RR_I", "MC_ROLLRATE_I"): ui_key = "Roll_I"
            elif k in ("ATC_RAT_RLL_D", "MC_RR_D", "MC_ROLLRATE_D"): ui_key = "Roll_D"
            elif k in ("ATC_RAT_PIT_P", "MC_PR_P", "MC_PITCHRATE_P"): ui_key = "Pitch_P"
            elif k in ("ATC_RAT_PIT_I", "MC_PR_I", "MC_PITCHRATE_I"): ui_key = "Pitch_I"
            elif k in ("ATC_RAT_PIT_D", "MC_PR_D", "MC_PITCHRATE_D"): ui_key = "Pitch_D"
            elif k in ("ATC_RAT_YAW_P", "MC_YR_P", "MC_YAWRATE_P"): ui_key = "Yaw_P"
            elif k in ("ATC_RAT_YAW_I", "MC_YR_I", "MC_YAWRATE_I"): ui_key = "Yaw_I"
            elif k in ("ATC_RAT_YAW_D", "MC_YR_D", "MC_YAWRATE_D"): ui_key = "Yaw_D"

            if ui_key:
                self.real_param_map[ui_key] = k 
                
                # 如果是第一次拿到非0参数，永久拍下安全快照
                if ui_key not in self.initial_params_snapshot and v != 0.0:
                    self.initial_params_snapshot[ui_key] = v

                
                self.ui_params[ui_key] = v
                self.param_updates[ui_key] = v


    def update_autotune_msg(self, text):
        with self._lock: self.autotune_status_msg = text

state = DroneState()

class MavlinkThread(threading.Thread):
    def __init__(self, local_port, target_ip, target_port):
        super().__init__()
        self.daemon = True; self.stop_event = threading.Event()
        self.local_port = int(local_port); self.target_ip = target_ip; self.target_port = int(target_port)
        self.master = None; self.pending_params = {}; self.param_lock = threading.Lock()

    def euler_to_quaternion(self, r, p, y):
        cr = math.cos(r * 0.5); sr = math.sin(r * 0.5); cp = math.cos(p * 0.5); sp = math.sin(p * 0.5); cy = math.cos(y * 0.5); sy = math.sin(y * 0.5)
        return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]

    def run(self):
        try:
            conn_str = f'udpin:0.0.0.0:{self.local_port}'
            state.add_log(f"开始监听: {conn_str}")
            self.master = mavutil.mavlink_connection(conn_str)
            self.master.mav.srcComponent = 1; self.master.mav.srcSystem = 255
            if hasattr(self.master, 'add_address'): self.master.add_address(self.target_ip, self.target_port)
            self.request_data_stream()
            threading.Timer(2.0, self.request_pid_params).start()
        except Exception as e:
            state.add_log(f"通信初始化失败: {e}"); return

        last_hb = 0; last_send = 0; last_req = 0; mode_chk = 0

        while not self.stop_event.is_set():
            try:
                while True:
                    msg = self.master.recv_match(blocking=False)
                    if not msg: break 

                    state.last_packet_time = time.time(); state.connected = True
                    mtype = msg.get_type()
                  
                    if mtype == 'HEARTBEAT':
                        if state.target_system != msg.get_srcSystem():
                            state.target_system = msg.get_srcSystem(); state.target_component = msg.get_srcComponent()
                            state.add_log(f"已锁定目标: SysID={state.target_system}")
                            threading.Thread(target=self.request_pid_params).start()
                        try:
                            if msg.custom_mode == 6: state.flight_mode = "OFFBOARD"
                            elif msg.custom_mode == 1: state.flight_mode = "MANUAL"
                            else: 
                                parsed = mavutil.mode_string_v10(msg)
                                state.flight_mode = parsed if parsed else f"Mode_{msg.custom_mode}"
                        except: state.flight_mode = f"Mode_{msg.custom_mode}"
                        state.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) > 0

                    elif mtype == 'ATTITUDE': state.roll, state.pitch, state.yaw = math.degrees(msg.roll), math.degrees(msg.pitch), math.degrees(msg.yaw)
                    elif mtype == 'SERVO_OUTPUT_RAW': state.servo_outputs[0:4] = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw]
                    elif mtype == 'SYS_STATUS': state.battery_voltage = msg.voltage_battery / 1000.0
                    elif mtype == 'PARAM_VALUE': state.update_param(msg.param_id, msg.param_value)
                    elif mtype == 'STATUSTEXT':
                        if "autotune" in msg.text.lower(): state.update_autotune_msg(msg.text); state.add_log(f"[FC] {msg.text}")
              
                now = time.time()
                if now - last_hb > 1.0: self.master.mav.heartbeat_send(6, 8, 0, 0, 0); last_hb = now
                if now - last_req > 3.0: self.request_data_stream(); last_req = now
              
                if state.send_enabled and (now - last_send > 0.05):
                    q = self.euler_to_quaternion(math.radians(state.target_roll), math.radians(state.target_pitch), math.radians(state.target_yaw))
                    self.master.mav.set_attitude_target_send(0, state.target_system, state.target_component, 0b00000111, q, 0, 0, 0, state.target_thrust)
                    last_send = now
                    if state.maintain_offboard and (now - mode_chk > 0.5):
                        if state.flight_mode != "OFFBOARD": self.force_offboard()
                        mode_chk = now
              
                self._process_param_queue()
                time.sleep(0.005) 
            except: pass

    def request_data_stream(self):
        if self.master:
            for s in [0, 3, 10, 11, 12]: self.master.mav.request_data_stream_send(state.target_system, state.target_component, s, 4, 1)

    def request_pid_params(self):
        if not self.master: return
        state.add_log(">>> 自动侦测并请求多版本固件 PID 参数...")
        possible_keys = [
            "MC_RR_P", "MC_RR_I", "MC_RR_D", "MC_PR_P", "MC_PR_I", "MC_PR_D", "MC_YR_P", "MC_YR_I", "MC_YR_D",
            "MC_ROLLRATE_P", "MC_ROLLRATE_I", "MC_ROLLRATE_D", "MC_PITCHRATE_P", "MC_PITCHRATE_I", "MC_PITCHRATE_D", "MC_YAWRATE_P", "MC_YAWRATE_I", "MC_YAWRATE_D",
            "ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_RAT_RLL_D", "ATC_RAT_PIT_P", "ATC_RAT_PIT_I", "ATC_RAT_PIT_D", "ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D"
        ]
        for p in possible_keys:
            self.master.mav.param_request_read_send(state.target_system, state.target_component, p.encode('utf-8'), -1)
            time.sleep(0.01)

    def set_param_async(self, name, value):
        with self.param_lock: self.pending_params[name] = value

    def _process_param_queue(self):
        item = None
        with self.param_lock:
            if self.pending_params:
                k = next(iter(self.pending_params)); v = self.pending_params.pop(k); item = (k, v)
        if item and self.master:
            self.master.mav.param_set_send(state.target_system, state.target_component, item[0].encode('utf-8'), item[1], mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def send_autotune_cmd(self, enable, axis_mask=0):
        if not self.master: return
        with state._lock: state.autotune_running = enable
        state.add_log(f"自整定指令: {'开启' if enable else '停止'} (Mask: {axis_mask})")
        self.master.mav.command_long_send(state.target_system, state.target_component, 211, 0, 1 if enable else 0, 1 if enable else 0, axis_mask, 0, 0, 0, 0)

    def handle_arm_switch(self, enable):
        if self.master:
            if enable:
                state.maintain_offboard = True; state.add_log("流程: 切换 OFFBOARD -> 解锁..."); self.force_offboard(); time.sleep(0.2); self.send_command_arm(True)
            else:
                state.maintain_offboard = False; state.add_log("流程: 切回 MANUAL -> 上锁..."); self.force_manual_mode(); time.sleep(0.1); self.send_command_arm(False)

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
        self.root.title("Quad 台架实验控制台")
        self.root.geometry("1750x1080")
        self.root.configure(bg=Theme.BG_APP)
      
        self.comm = None
        self.pid_sliders = {}
        self.user_interacting_pid = False
        self.active_tab = 'console' 
        self.plot_counter = 0 

        self.build_navigation()

        self.main_container = tk.Frame(self.root, bg=Theme.BG_APP)
        self.main_container.pack(fill=tk.BOTH, expand=True)

        self.pages = {}
        self.pages['console'] = tk.Frame(self.main_container, bg=Theme.BG_APP)
        self.pages['tune'] = tk.Frame(self.main_container, bg=Theme.BG_APP)

        self.build_page_console(self.pages['console'])
        self.build_page_tune(self.pages['tune'])

        self.switch_page('console')
        self.update_loop()

    def create_card(self, parent, title="", pad=8): 
        outer = tk.Frame(parent, bg=Theme.BORDER, padx=1, pady=1)
        outer.pack(fill=tk.BOTH, expand=True, padx=6, pady=4) 
        inner = tk.Frame(outer, bg=Theme.BG_CARD, padx=pad, pady=pad)
        inner.pack(fill=tk.BOTH, expand=True)
        if title:
            tk.Label(inner, text=title, bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, font=("微软雅黑", 10, "bold")).pack(anchor='w', pady=(0, 2))
        return inner

    def build_navigation(self):
        nav = tk.Frame(self.root, bg=Theme.BG_APP, height=45)
        nav.pack(side=tk.TOP, fill=tk.X); nav.pack_propagate(False)

        tk.Label(nav, text="❖ Quad GCS 实验平台", font=("微软雅黑", 13, "bold"), fg=Theme.COLOR_ROLL, bg=Theme.BG_APP).pack(side=tk.LEFT, padx=20)
        self.nav_btns = {}
        btn_box = tk.Frame(nav, bg=Theme.BG_APP)
        btn_box.pack(side=tk.LEFT, padx=40)
      
        for key, text in [('console', "⛐ 主控台 (连接与控制)"), ('tune', "⚙ PID 智能整定")]:
            btn = tk.Button(btn_box, text=text, font=("微软雅黑", 10, "bold"), bg=Theme.BG_APP, fg=Theme.TEXT_SUB, 
                            bd=0, activebackground=Theme.BG_CARD, activeforeground=Theme.COLOR_ROLL, cursor="hand2",
                            command=lambda k=key: self.switch_page(k))
            btn.pack(side=tk.LEFT, padx=10, pady=5)
            self.nav_btns[key] = btn

        self.btn_export = tk.Button(nav, text="↓ 导出数据", font=("微软雅黑", 10, "bold"), bg="#00bcd4", fg="#000000", bd=0, cursor="hand2", padx=10, pady=2, command=self.export_csv)
        self.btn_export.pack(side=tk.RIGHT, padx=30, pady=8)

        tk.Frame(self.root, bg=Theme.BORDER, height=1).pack(fill=tk.X)

    def switch_page(self, target_key):
        self.active_tab = target_key
        for k, btn in self.nav_btns.items():
            btn.config(fg=Theme.COLOR_ROLL if k == target_key else Theme.TEXT_SUB)
        for p in self.pages.values(): p.pack_forget()
        self.pages[target_key].pack(fill=tk.BOTH, expand=True)

    def build_page_console(self, page):
        left = tk.Frame(page, bg=Theme.BG_APP, width=320)
        left.pack(side=tk.LEFT, fill=tk.Y, pady=2)
        left.pack_propagate(False) 

        c_stat = self.create_card(left)
        top_bar = tk.Frame(c_stat, bg=Theme.BG_CARD); top_bar.pack(fill=tk.X)
        tk.Label(top_bar, text="◷ 通信状态监控", fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("微软雅黑", 9, "bold")).pack(side=tk.LEFT)
        self.lbl_latency_top = tk.Label(top_bar, text="-- ms", bg="#0a3a40", fg=Theme.COLOR_ROLL, font=("Consolas", 8, "bold"), padx=8)
        self.lbl_latency_top.pack(side=tk.RIGHT)
      
        box = tk.Frame(c_stat, bg=Theme.BG_BOX, highlightbackground=Theme.BORDER, highlightthickness=1)
        box.pack(fill=tk.X, pady=4)
        inner_f = tk.Frame(box, bg=Theme.BG_BOX); inner_f.pack(fill=tk.X, padx=10, pady=8)
        self.cv_dot = tk.Canvas(inner_f, width=12, height=12, bg=Theme.BG_BOX, highlightthickness=0); self.cv_dot.pack(side=tk.LEFT, padx=(0, 8))
        self.dot_id = self.cv_dot.create_oval(1, 1, 11, 11, fill=Theme.COLOR_DANGER, outline="")
      
        link_text_f = tk.Frame(inner_f, bg=Theme.BG_BOX); link_text_f.pack(side=tk.LEFT, fill=tk.X)
        self.lbl_big_link = tk.Label(link_text_f, text="离线 (OFFLINE)", font=("微软雅黑", 11, "bold"), fg=Theme.COLOR_DANGER, bg=Theme.BG_BOX)
        self.lbl_big_link.pack(anchor='w')
        self.lbl_ip_info = tk.Label(link_text_f, text="--:--", fg=Theme.TEXT_SUB, bg=Theme.BG_BOX, font=("Consolas", 8))
        self.lbl_ip_info.pack(anchor='w')
      
        grid_f = tk.Frame(c_stat, bg=Theme.BG_CARD); grid_f.pack(fill=tk.X, pady=2)
        def add_sys_info(r, c, title, attr):
            f = tk.Frame(grid_f, bg=Theme.BG_CARD); f.grid(row=r, column=c, sticky="w", padx=15, pady=2)
            tk.Label(f, text=title, fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("Consolas", 7, "bold")).pack(anchor='w')
            lbl = tk.Label(f, text="--", fg=Theme.TEXT_MAIN, bg=Theme.BG_CARD, font=("Consolas", 10, "bold"))
            lbl.pack(anchor='w'); setattr(self, attr, lbl)
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
        self.send_var = tk.BooleanVar()
        self.chk_send = tk.Checkbutton(c_ctrl, text="[1] 允许发送指令", variable=self.send_var, bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, 
                                       selectcolor=Theme.BG_APP, activebackground=Theme.BG_CARD, font=("微软雅黑", 9), 
                                       command=self.on_send_switch, state="disabled")
        self.chk_send.pack(anchor='w')
        self.arm_var = tk.BooleanVar()
        self.chk_arm = tk.Checkbutton(c_ctrl, text="[2] 允许解锁电机 (ARM)", variable=self.arm_var, bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, 
                                      selectcolor=Theme.BG_APP, activebackground=Theme.BG_CARD, font=("微软雅黑", 9), 
                                      command=self.toggle_arm, state="disabled")
        self.chk_arm.pack(anchor='w')

        c_setpoint = self.create_card(left, "飞行控制 (SETPOINTS)")
        self.create_canvas_slider(c_setpoint, "Roll", -30, 30, 'roll', Theme.COLOR_ROLL)
        self.create_canvas_slider(c_setpoint, "Pitch", -30, 30, 'pitch', Theme.COLOR_PITCH)
        self.create_canvas_slider(c_setpoint, "Yaw", -180, 180, 'yaw', Theme.COLOR_YAW)
        self.create_canvas_slider(c_setpoint, "THR", 1100, 1900, 'thrust', Theme.COLOR_THRUST)

        right = tk.Frame(page, bg=Theme.BG_APP)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=2)

        top_f = tk.Frame(right, bg=Theme.BG_APP, height=80)
        top_f.pack(fill=tk.X); top_f.pack_propagate(False) 
        self.lbl_v_roll = self.create_metric_card(top_f, "ROLL 滚转", Theme.COLOR_ROLL)
        self.lbl_v_pitch = self.create_metric_card(top_f, "PITCH 俯仰", Theme.COLOR_PITCH)
        self.lbl_v_yaw = self.create_metric_card(top_f, "YAW 偏航", Theme.COLOR_YAW)

        mid_f = tk.Frame(right, bg=Theme.BG_APP)
        mid_f.pack(fill=tk.BOTH, expand=True)
        c_chart = self.create_card(mid_f, "Telemetry 动态波形")
        self.setup_2x2_charts(c_chart)

        btm_show_f = tk.Frame(right, bg=Theme.BG_APP, height=260)
        btm_show_f.pack(fill=tk.X)
        btm_show_f.pack_propagate(False)

        pwm_f = tk.Frame(btm_show_f, bg=Theme.BG_APP)
        pwm_f.place(relx=0, rely=0, relwidth=0.45, relheight=1.0)
        c_pwm = self.create_card(pwm_f, "电机 PWM 输出")
        self.pwm_bars = []
        colors = [Theme.COLOR_ROLL, Theme.COLOR_PITCH, Theme.COLOR_YAW, Theme.COLOR_THRUST]
        for i, color in enumerate(colors):
            row = tk.Frame(c_pwm, bg=Theme.BG_CARD); row.pack(fill=tk.X, pady=7) 
            tk.Label(row, text=f"M{i+1}", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("Consolas", 9, "bold"), width=3).pack(side=tk.LEFT)
            cv = tk.Canvas(row, height=12, bg=Theme.BG_INPUT, highlightthickness=0)
            cv.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=8)
            cv.create_rectangle(0, 0, 9999, 12, fill="#1c253c", outline="")
            rect = cv.create_rectangle(0, 0, 0, 12, fill=color, outline="")
            lbl = tk.Label(row, text="1000", bg=Theme.BG_CARD, fg=Theme.TEXT_MAIN, font=("Consolas", 10), width=4)
            lbl.pack(side=tk.RIGHT); self.pwm_bars.append((cv, rect, lbl))

        att_f = tk.Frame(btm_show_f, bg=Theme.BG_APP)
        att_f.place(relx=0.45, rely=0, relwidth=0.55, relheight=1.0)
        c_att = self.create_card(att_f, "姿态球 (Attitude Indicator)")
        self.cv_att = tk.Canvas(c_att, bg=Theme.BG_CARD, highlightthickness=0)
        self.cv_att.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)
        self.cv_att.bind("<Configure>", lambda e: self.draw_attitude(self.cv_att, state.roll, state.pitch))

        log_f = tk.Frame(right, bg=Theme.BG_APP, height=100)
        log_f.pack(fill=tk.X)
        log_f.pack_propagate(False) 
        c_log = self.create_card(log_f, "系统日志", pad=5)
        self.log_text = ScrolledText(c_log, bg=Theme.BG_INPUT, fg=Theme.TEXT_SUB, bd=0, font=("Consolas", 8))
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def create_metric_card(self, parent, title, color):
        outer = tk.Frame(parent, bg=Theme.BORDER, padx=1, pady=1)
        outer.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)
        inner = tk.Frame(outer, bg=Theme.BG_CARD, padx=10, pady=2)
        inner.pack(fill=tk.BOTH, expand=True)
        tk.Label(inner, text=title, fg=Theme.TEXT_SUB, bg=Theme.BG_CARD, font=("微软雅黑", 9)).pack()
        lbl = tk.Label(inner, text="0.0", fg=color, bg=Theme.BG_CARD, font=("Consolas", 24, "bold"))
        lbl.pack()
        return lbl

    def create_canvas_slider(self, p, lbl, min_v, max_v, tag, color):
        f = tk.Frame(p, bg=Theme.BG_CARD)
        f.pack(fill=tk.X, pady=6)
      
        h = tk.Frame(f, bg=Theme.BG_CARD)
        h.pack(fill=tk.X)
        tk.Label(h, text=lbl, bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("Consolas", 10)).pack(side=tk.LEFT)
        v_lbl = tk.Label(h, text="0.0", bg=Theme.BG_CARD, fg=color, font=("Consolas", 10, "bold"))
        v_lbl.pack(side=tk.RIGHT)
      
        cv = tk.Canvas(f, height=24, bg=Theme.BG_CARD, highlightthickness=0, cursor="hand2")
        cv.pack(fill=tk.X, pady=(4, 2))
      
        self.slider_data = {
            'val': 1100.0 if tag == 'thrust' else 0.0,
            'pos_x': 0
        }
        def update_logic(val):
            v_lbl.config(text=f"{val:.1f}")
            if tag == 'roll': state.target_roll = val
            elif tag == 'pitch': state.target_pitch = val
            elif tag == 'yaw': state.target_yaw = val
            elif tag == 'thrust': state.target_thrust = (val - 1000) / 1000.0
        def render(event=None):
            cv.delete("all")
            w = cv.winfo_width()
            if w < 30: return
          
            track_h = 14
            track_y = (24 - track_h) // 2
            cv.create_rectangle(0, track_y, w, track_y + track_h, fill=Theme.BG_BOX, outline=Theme.BORDER)
          
            curr_val = 1100.0 if tag == 'thrust' else (state.target_roll if tag == 'roll' else (state.target_pitch if tag == 'pitch' else state.target_yaw))
            if tag == 'thrust' and not self.arm_var.get(): 
                curr_val = 1100.0 
              
            if event:
                raw_x = event.x - 12
                raw_x = max(0, min(raw_x, w - 24))
                val = (raw_x / (w - 24)) * (max_v - min_v) + min_v
                update_logic(val)
                pos_x = raw_x
            else:
                pos_x = ((curr_val - min_v) / (max_v - min_v)) * (w - 24)
          
            hx, hy, hw, hh = pos_x, 2, 24, 20
            cv.create_rectangle(hx, hy, hx + hw, hy + hh, fill="#4a4a4a", outline="#222")
            cv.create_line(hx + 12, hy + 4, hx + 12, hy + 16, fill="#333")
            cv.create_line(hx + 11, hy + 4, hx + 11, hy + 16, fill="#555")
        def on_click(event):
            render(event)
          
        cv.bind("<B1-Motion>", render)
        cv.bind("<Button-1>", on_click)
        cv.bind("<Configure>", lambda e: f.after(10, render))
      
        if tag == 'thrust': self.thrust_cv = cv; self.thrust_render = render
        return cv

    def setup_2x2_charts(self, parent):
        self.fig_status = Figure(figsize=(10, 2), facecolor=Theme.BG_CARD)
        self.fig_status.subplots_adjust(left=0.04, right=0.98, top=0.85, bottom=0.15, wspace=0.12, hspace=0.45)
        self.ax_status = []; self.line_status = []
        self.x_data = np.arange(100)
        self.y_data = [np.zeros(100) for _ in range(4)]
      
        configs = [
            ("ROLL 滚转", Theme.COLOR_ROLL, -40, 40, [-40, -20, 0, 20, 40]), 
            ("PITCH 俯仰", Theme.COLOR_PITCH, -40, 40, [-40, -20, 0, 20, 40]), 
            ("YAW 偏航", Theme.COLOR_YAW, -180, 180, [-180, -90, 0, 90, 180]), 
            ("THRUST 推力", Theme.COLOR_THRUST, 0, 1.0, [0, 0.2, 0.4, 0.6, 0.8, 1.0])
        ]
      
        for i, (title, color, ymin, ymax, yticks) in enumerate(configs):
            ax = self.fig_status.add_subplot(2, 2, i+1, facecolor=Theme.BG_APP)
            ax.set_title(title, color=Theme.TEXT_MAIN, fontsize=9, loc='left')
            ax.tick_params(colors=Theme.TEXT_SUB, labelsize=7)
            for spine in ax.spines.values(): spine.set_color(Theme.BORDER)
            ax.grid(True, color=Theme.BORDER, linestyle='-', linewidth=0.5)
          
            ax.set_ylim(ymin, ymax)
            ax.set_yticks(yticks) 
          
            l, = ax.plot(self.x_data, self.y_data[i], color=color, lw=1.5)
            self.ax_status.append(ax); self.line_status.append(l)

        self.canvas_status = FigureCanvasTkAgg(self.fig_status, master=parent)
        self.canvas_status.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def draw_attitude(self, cv, roll, pitch):
        cv.delete("all")
        w = cv.winfo_width(); h = cv.winfo_height()
        if w < 50 or h < 50: return
        cx, cy = w / 2, h / 2
        R = min(w, h) / 2 - 25 
      
        cv.create_rectangle(0, 0, w, h, fill=Theme.BG_CARD, outline="")

        pitch_px = pitch * 2.0
        pitch_px = max(min(pitch_px, R), -R) 
        angle = math.radians(-roll) 
        L = R * 3 
      
        y_center = cy + pitch_px
        dx = L * math.cos(angle); dy = L * math.sin(angle)
        x1 = cx - dx; y1 = y_center + dy
        x2 = cx + dx; y2 = y_center - dy
      
        cv.create_polygon(x1, y1, x2, y2, cx+L, cy-L, cx-L, cy-L, fill="#1e5f8f", outline="") 
        cv.create_polygon(x1, y1, x2, y2, cx+L, cy+L, cx-L, cy+L, fill="#6e421c", outline="") 
      
        cv.create_line(x1, y1, x2, y2, fill="white", width=2)
      
        for deg in [-30, -20, -10, 10, 20, 30]:
            offset = deg * 2.0
            l_y = y_center - offset
            line_w = 20 if deg % 20 == 0 else 10 
          
            lx1 = cx - line_w * math.cos(angle) + offset * math.sin(angle)
            ly1 = l_y + line_w * math.sin(angle) - offset * (1 - math.cos(angle))
            lx2 = cx + line_w * math.cos(angle) + offset * math.sin(angle)
            ly2 = l_y - line_w * math.sin(angle) - offset * (1 - math.cos(angle))
          
            mid_x = (lx1+lx2)/2; mid_y = (ly1+ly2)/2
            cv.create_line(lx1, ly1, mid_x - 5*math.cos(angle), mid_y + 5*math.sin(angle), fill="white", width=1)
            cv.create_line(mid_x + 5*math.cos(angle), mid_y - 5*math.sin(angle), lx2, ly2, fill="white", width=1)

        mask_w = max(w, h)
        cv.create_oval(cx-R-mask_w, cy-R-mask_w, cx+R+mask_w, cy+R+mask_w, outline=Theme.BG_CARD, width=mask_w*2)
        cv.create_oval(cx-R, cy-R, cx+R, cy+R, outline=Theme.COLOR_ROLL, width=2) 
      
        arc_R = R + 12
        cv.create_arc(cx-arc_R, cy-arc_R, cx+arc_R, cy+arc_R, start=30, extent=120, style=tk.ARC, outline="white", width=1)
        for r_deg in [-60, -45, -30, -15, 0, 15, 30, 45, 60]:
            r_rad = math.radians(r_deg - 90)
            tx1 = cx + arc_R * math.cos(r_rad); ty1 = cy + arc_R * math.sin(r_rad)
            tx2 = cx + (arc_R+6) * math.cos(r_rad); ty2 = cy + (arc_R+6) * math.sin(r_rad)
            cv.create_line(tx1, ty1, tx2, ty2, fill="white", width=1 if r_deg % 30 != 0 else 2)
      
        ptr_rad = math.radians(roll - 90)
        px1 = cx + (arc_R-2) * math.cos(ptr_rad); py1 = cy + (arc_R-2) * math.sin(ptr_rad)
        px2 = cx + (arc_R-10) * math.cos(ptr_rad-0.1); py2 = cy + (arc_R-10) * math.sin(ptr_rad-0.1)
        px3 = cx + (arc_R-10) * math.cos(ptr_rad+0.1); py3 = cy + (arc_R-10) * math.sin(ptr_rad+0.1)
        cv.create_polygon(px1, py1, px2, py2, px3, py3, fill=Theme.COLOR_ROLL, outline="")

        ret_c = "#ffb000"
        cv.create_line(cx-18, cy, cx-6, cy, fill=ret_c, width=2)
        cv.create_line(cx-6, cy, cx-6, cy+6, fill=ret_c, width=2)
        cv.create_line(cx+6, cy, cx+18, cy, fill=ret_c, width=2)
        cv.create_line(cx+6, cy, cx+6, cy+6, fill=ret_c, width=2)
        cv.create_oval(cx-2, cy-2, cx+2, cy+2, fill=ret_c, outline="")

    def build_page_tune(self, page):
        self.root.option_add('*TCombobox*Listbox.background', Theme.BG_INPUT)
        self.root.option_add('*TCombobox*Listbox.foreground', Theme.TEXT_MAIN)
        style = ttk.Style()
        style.theme_use('default')
        style.configure("TCombobox", fieldbackground=Theme.BG_INPUT, background=Theme.BG_INPUT, foreground=Theme.TEXT_MAIN, borderwidth=0)

        top_step = tk.Frame(page, bg=Theme.BG_APP)
        top_step.pack(fill=tk.X, pady=(10, 0), padx=10)
        self.step_labels = []
        for i, s in enumerate(["① 待机", "② 激励", "③ 采集", "④ 辨识", "⑤ 优化", "⑥ 完成"]):
            f = tk.Frame(top_step, bg=Theme.BORDER, padx=1, pady=1)
            f.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
            lbl = tk.Label(f, text=s, bg=Theme.BG_CARD, fg=Theme.TEXT_SUB if i>0 else Theme.COLOR_ROLL, font=("微软雅黑", 10, "bold"), pady=10)
            lbl.pack(fill=tk.BOTH, expand=True)
            self.step_labels.append(lbl)

        left = tk.Frame(page, bg=Theme.BG_APP, width=320); left.pack(side=tk.LEFT, fill=tk.Y, pady=10)
        right = tk.Frame(page, bg=Theme.BG_APP); right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, pady=10)

        c_at = self.create_card(left, "", pad=12)
        
        lbl_f = tk.Frame(c_at, bg=Theme.BG_CARD); lbl_f.pack(fill=tk.X, pady=(0, 5))
        tk.Label(lbl_f, text="◎", fg=Theme.COLOR_ROLL, bg=Theme.BG_CARD, font=("Consolas", 10)).pack(side=tk.LEFT)
        tk.Label(lbl_f, text=" 整定目标轴", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("微软雅黑", 10, "bold")).pack(side=tk.LEFT)
        
        self.tune_axis_var = tk.StringVar(value="1")
        ax_f = tk.Frame(c_at, bg=Theme.BG_CARD); ax_f.pack(fill=tk.X, pady=(0, 15))
        for txt, val, col in [("Roll", "1", Theme.COLOR_ROLL), ("Pitch", "2", Theme.TEXT_MAIN), ("Yaw", "4", Theme.TEXT_MAIN), ("全部", "7", Theme.TEXT_MAIN)]:
            tk.Radiobutton(ax_f, text=txt, variable=self.tune_axis_var, value=val, bg=Theme.BG_CARD, fg=col, selectcolor=Theme.BG_APP, activebackground=Theme.BG_CARD, font=("Consolas", 9, "bold")).pack(side=tk.LEFT, expand=True)

        tk.Label(c_at, text="阶跃测试幅度 (DEG)", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("微软雅黑", 9, "bold")).pack(anchor='w', pady=(5,2))
        self.ent_step_amp = tk.Entry(c_at, bg=Theme.BG_APP, fg=Theme.TEXT_MAIN, bd=0, font=("Consolas", 11))
        self.ent_step_amp.insert(0, "15")
        self.ent_step_amp.pack(fill=tk.X, ipady=5)
        
        tk.Label(c_at, text="整定激进程度 (Agility)", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("微软雅黑", 9, "bold")).pack(anchor='w', pady=(10,2))
        self.agility_var = tk.StringVar(value="适中 (0.075)")
        cb = ttk.Combobox(c_at, textvariable=self.agility_var, values=["保守 (0.050)", "适中 (0.075)", "激进 (0.100)"], state="readonly", font=("微软雅黑", 10))
        cb.pack(fill=tk.X, pady=2, ipady=3)
        
        tk.Frame(c_at, bg=Theme.BORDER, height=1).pack(fill=tk.X, pady=15)

        self.btn_autotune = tk.Button(c_at, text="▷ 开始自动整定", bg=Theme.COLOR_ROLL, fg=Theme.BG_APP, font=("微软雅黑", 11, "bold"), bd=0, cursor="hand2", command=self.toggle_autotune)
        self.btn_autotune.pack(fill=tk.X, pady=(0, 8), ipady=6)
        
        self.btn_stop_tune = tk.Button(c_at, text="□ 停止整定", bg="#2a1b22", fg=Theme.COLOR_DANGER, font=("微软雅黑", 10), bd=0, cursor="hand2", command=self.stop_tune)
        self.btn_stop_tune.pack(fill=tk.X, pady=2, ipady=4)
        
        self.btn_step_test = tk.Button(c_at, text="⚡ 运行阶跃性能测试", bg="#182c25", fg=Theme.COLOR_THRUST, font=("微软雅黑", 10, "bold"), bd=0, cursor="hand2", command=self.run_step_test_thread)
        self.btn_step_test.pack(fill=tk.X, pady=(8, 10), ipady=6)

        c_perf = self.create_card(left)
        lbl_p = tk.Frame(c_perf, bg=Theme.BG_CARD); lbl_p.pack(fill=tk.X, pady=(0, 5))
        tk.Label(lbl_p, text="∿", fg=Theme.COLOR_ROLL, bg=Theme.BG_CARD, font=("Consolas", 10)).pack(side=tk.LEFT)
        tk.Label(lbl_p, text=" 性能指标 ", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("微软雅黑", 10, "bold")).pack(side=tk.LEFT)
        
        perf_grid = tk.Frame(c_perf, bg=Theme.BG_CARD); perf_grid.pack(fill=tk.BOTH, expand=True)
        self.perf_labels = {}
        
        metrics = [
            ("超调量", "%", Theme.COLOR_PITCH), ("上升时间", "ms", Theme.COLOR_ROLL), ("稳态误差", "deg", Theme.COLOR_THRUST),
            ("峰值时间", "ms", Theme.COLOR_PITCH), ("调节时间", "ms", Theme.COLOR_ROLL), ("稳态值", "deg", Theme.COLOR_THRUST)
        ]
        
        for i, (title, unit, color) in enumerate(metrics):
            r, c = i // 3, i % 3
            f = tk.Frame(perf_grid, bg=Theme.BG_BOX, padx=10, pady=12) 
            f.grid(row=r, column=c, padx=5, pady=5, sticky='nsew')
            tk.Label(f, text=title, bg=Theme.BG_BOX, fg=Theme.TEXT_SUB, font=("微软雅黑", 8)).pack()
            lbl = tk.Label(f, text="-", bg=Theme.BG_BOX, fg=color, font=("Consolas", 14, "bold")); lbl.pack(pady=4)
            tk.Label(f, text=unit, bg=Theme.BG_BOX, fg=Theme.TEXT_SUB, font=("Consolas", 8)).pack()
            self.perf_labels[title] = lbl

        c_chart = self.create_card(right, "姿态响应曲线")
        self.fig_tune = Figure(figsize=(8, 2.3), facecolor=Theme.BG_CARD)
        self.ax_tune = self.fig_tune.add_subplot(111, facecolor=Theme.BG_APP)
        self.ax_tune.grid(True, color=Theme.BORDER, linestyle='--', alpha=0.5); self.ax_tune.tick_params(colors=Theme.TEXT_SUB, labelsize=8); self.ax_tune.set_ylim(-5, 25)
        self.line_target, = self.ax_tune.plot([], [], color=Theme.TEXT_SUB, linestyle=':', label="测试期望")
        self.line_before, = self.ax_tune.plot([], [], color=Theme.COLOR_DANGER, lw=1.5, label="自整定抽搐波形")
        self.line_after, = self.ax_tune.plot([], [], color=Theme.COLOR_THRUST, lw=2, label="阶跃响应")
        self.ax_tune.legend(facecolor=Theme.BG_CARD, edgecolor=Theme.BORDER, labelcolor=Theme.TEXT_MAIN, fontsize=8)
        self.canvas_tune = FigureCanvasTkAgg(self.fig_tune, master=c_chart)
        self.canvas_tune.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        c_table = self.create_card(right, "📄 整定前后参数对比")
        h_f = tk.Frame(c_table, bg=Theme.BG_CARD); h_f.pack(fill=tk.X, pady=(0, 10))
        tk.Label(h_f, text="轴 / 参数", bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("微软雅黑", 9), width=15, anchor='w').pack(side=tk.LEFT, padx=10)
        for p in ["KP", "KI", "KD"]:
            tk.Label(h_f, text=p, bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("Consolas", 10), width=15).pack(side=tk.LEFT, expand=True)

        self.comp_labels = {}
        axes_info = [("Roll 横滚", Theme.COLOR_ROLL, "Roll"), 
                     ("Pitch 俯仰", Theme.COLOR_PITCH, "Pitch"), 
                     ("Yaw 偏航", Theme.COLOR_YAW, "Yaw")]

        for ax_name, color, prefix in axes_info:
            self.comp_labels[prefix] = {'P': {}, 'I': {}, 'D': {}}
            b_f = tk.Frame(c_table, bg=Theme.BG_CARD); b_f.pack(fill=tk.X, pady=(5, 2))
            tk.Label(b_f, text=ax_name, bg=Theme.BG_BOX, fg=color, font=("微软雅黑", 9, "bold"), padx=8, pady=2, highlightbackground=color, highlightthickness=1).pack(side=tk.LEFT, padx=10)

            r1 = tk.Frame(c_table, bg=Theme.BG_BOX); r1.pack(fill=tk.X, pady=1, padx=10)
            tk.Label(r1, text="整定前", bg=Theme.BG_BOX, fg=Theme.TEXT_SUB, font=("微软雅黑", 9), width=12, anchor='w').pack(side=tk.LEFT, padx=10, pady=6)
            for p in ["P", "I", "D"]:
                lbl = tk.Label(r1, text="0.000", bg=Theme.BG_BOX, fg=Theme.TEXT_SUB, width=15, font=("Consolas", 10))
                lbl.pack(side=tk.LEFT, expand=True); self.comp_labels[prefix][p]['old'] = lbl

            r2 = tk.Frame(c_table, bg=Theme.BG_BOX); r2.pack(fill=tk.X, pady=(1, 10), padx=10)
            tk.Label(r2, text="整定后", bg=Theme.BG_BOX, fg=Theme.COLOR_THRUST, font=("微软雅黑", 9, "bold"), width=12, anchor='w').pack(side=tk.LEFT, padx=10, pady=6)
            for p in ["P", "I", "D"]:
                lbl = tk.Label(r2, text="-", bg=Theme.BG_BOX, fg=Theme.COLOR_THRUST, width=15, font=("Consolas", 11, "bold"))
                lbl.pack(side=tk.LEFT, expand=True); self.comp_labels[prefix][p]['new'] = lbl

        # --- 右 3：手动微调覆盖 ---
        c_man = self.create_card(right, "📝 手动调参")
        badge_f = tk.Frame(c_man, bg=Theme.BG_CARD); badge_f.pack(fill=tk.X, pady=(0, 5))
        entry_f = tk.Frame(c_man, bg=Theme.BG_CARD); entry_f.pack(fill=tk.X)
        
        self.manual_entries = {}
        for ax_name, color, prefix in axes_info:
            ax_frame = tk.Frame(badge_f, bg=Theme.BG_CARD); ax_frame.pack(side=tk.LEFT, expand=True, fill=tk.X)
            tk.Label(ax_frame, text=ax_name.split()[0], bg=Theme.BG_BOX, fg=color, font=("Consolas", 9, "bold"), padx=10, pady=2, highlightbackground=color, highlightthickness=1).pack(anchor='w', padx=15)
            
            p_frame = tk.Frame(entry_f, bg=Theme.BG_CARD); p_frame.pack(side=tk.LEFT, expand=True, fill=tk.X)
            
            for lbl_txt, p_suf in [("Kp", "P"), ("Ki", "I"), ("Kd", "D")]:
                cell = tk.Frame(p_frame, bg=Theme.BG_CARD); cell.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=5)
                tk.Label(cell, text=lbl_txt, bg=Theme.BG_CARD, fg=Theme.TEXT_SUB, font=("Consolas", 8)).pack(anchor='center')
                
                # 【新增：使用 Spinbox 替代原 Entry，自带上下调整按钮】
                step_val = 0.002 if p_suf == "D" else 0.01 
                ent = tk.Spinbox(cell, from_=0.0, to=10.0, increment=step_val, format="%.4f",
                                 bg=Theme.BG_APP, fg=Theme.TEXT_MAIN, bd=0, justify='center', 
                                 font=("Consolas", 10, "bold"), buttonbackground=Theme.BG_INPUT)
                ent.delete(0, tk.END)
                ent.insert(0, "0.0000")
                ent.pack(fill=tk.X, ipady=4, pady=2)
                
                ui_key = f"{prefix}_{p_suf}"
                self.manual_entries[ui_key] = ent

        bot_btn_f = tk.Frame(c_man, bg=Theme.BG_CARD); bot_btn_f.pack(fill=tk.X, pady=(15, 5), padx=10)
        tk.Button(bot_btn_f, text="写入参数", bg=Theme.COLOR_ROLL, fg=Theme.BG_APP, font=("微软雅黑", 10, "bold"), bd=0, padx=15, pady=4, cursor="hand2", command=self.write_manual_params).pack(side=tk.LEFT, padx=(0, 10))
        tk.Button(bot_btn_f, text="刷新读取", bg=Theme.BG_INPUT, fg=Theme.TEXT_SUB, font=("微软雅黑", 10), bd=0, padx=15, pady=4, cursor="hand2", command=self.refresh_pids).pack(side=tk.LEFT, padx=(0, 10))
        # 【新增：安全回滚的后悔药按钮】
        tk.Button(bot_btn_f, text="撤销修改(恢复初值)", bg="#331c22", fg=Theme.COLOR_DANGER, font=("微软雅黑", 10), bd=0, padx=15, pady=4, cursor="hand2", command=self.restore_initial_params).pack(side=tk.LEFT)

    def toggle_conn(self):
        if not self.comm:
            try:
                self.comm = MavlinkThread(self.ent_local.get(), self.ent_ip.get(), self.ent_port.get())
                self.comm.start()
                self.btn_conn.config(text="断开连接", bg=Theme.COLOR_DANGER, fg=Theme.TEXT_MAIN)
                self.chk_send.config(state="normal")
                with state._lock: state.telemetry_data.clear()
            except Exception as e: state.add_log(f"错误: {e}")
        else:
            self.comm.stop(); self.comm.join(); self.comm = None; state.connected = False
            self.btn_conn.config(text="启 动 连 接", bg=Theme.COLOR_DANGER, fg=Theme.TEXT_MAIN) 
            self.chk_send.config(state="disabled")
            self.send_var.set(False); self.on_send_switch()

    def on_send_switch(self):
        if self.send_var.get():
            state.send_enabled = True; self.chk_arm.config(state="normal"); state.add_log("指令流已启动 (Setpoints Only)")
        else:
            state.send_enabled = False
            if self.comm:
                state.maintain_offboard = False; self.comm.force_manual_mode(); self.chk_arm.config(state="disabled"); self.arm_var.set(False); state.add_log("指令流停止 -> 强制切 MANUAL")

    def toggle_arm(self):
        if self.comm: self.comm.handle_arm_switch(self.arm_var.get())

    def export_csv(self):
        if not state.telemetry_data:
            messagebox.showinfo("提示", "当前无数据可导出！请先连接飞控获取数据。"); return
        fn = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=f"telemetry_{int(time.time())}.csv", filetypes=[("CSV 文件", "*.csv")])
        if fn:
            try:
                with open(fn, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(['time', 'roll', 'pitch', 'yaw', 'thrust', 'm1', 'm2', 'm3', 'm4'])
                    with state._lock: writer.writerows(state.telemetry_data)
                messagebox.showinfo("导出成功", f"成功导出 {len(state.telemetry_data)} 条数据！")
            except Exception as e: messagebox.showerror("导出失败", f"文件保存失败：\n{e}")

    def toggle_autotune(self):
        if not self.comm: return
        with state._lock: is_running = state.autotune_running
        if not is_running:
            with state._lock: state.old_params = state.ui_params.copy()
            aggr_str = self.agility_var.get()
            aggr_val = 0.075
            if "0.05" in aggr_str: aggr_val = 0.05
            elif "0.1" in aggr_str: aggr_val = 0.1
            self.comm.set_param_async("AUTOTUNE_AGGR", aggr_val)
            axis_val = self.tune_axis_var.get()
            axis = 7 if axis_val == "7" else int(axis_val)
            self.comm.send_autotune_cmd(True, axis)
            self.btn_autotune.config(text="执行中...", state=tk.DISABLED)
            for l in getattr(self, 'step_labels', []): l.config(fg=Theme.TEXT_SUB)
            if hasattr(self, 'step_labels') and len(self.step_labels) > 1: self.step_labels[1].config(fg=Theme.COLOR_ROLL)
            self.update_param_table() 
            with state._lock:
                state.test_time_buffer.clear()
                state.test_actual_buffer.clear()
            threading.Thread(target=self.live_autotune_plot).start()
        else:
            self.stop_tune()

    def stop_tune(self):
        if self.comm: self.comm.send_autotune_cmd(False, 0)
        state.autotune_running = False
        self.reset_autotune_ui()

    def live_autotune_plot(self):
        self.line_target.set_data([], [])
        self.line_after.set_data([], [])
        start_time = time.time()
        axis_mode = self.tune_axis_var.get()
        while state.autotune_running:
            current_time = time.time() - start_time
            with state._lock:
                act = state.roll if axis_mode == "1" else state.pitch if axis_mode == "2" else state.yaw
                state.test_time_buffer.append(current_time)
                state.test_actual_buffer.append(act)
                if len(state.test_time_buffer) > 200:
                    state.test_time_buffer.pop(0)
                    state.test_actual_buffer.pop(0)
                times = list(state.test_time_buffer)
                actuals = list(state.test_actual_buffer)
            self.line_before.set_data(times, actuals)
            if times:
                self.ax_tune.set_xlim(times[0], times[-1] + 1)
                min_y, max_y = min(actuals) - 5, max(actuals) + 5
                self.ax_tune.set_ylim(min_y if min_y < -5 else -5, max_y if max_y > 25 else 25)
            self.canvas_tune.draw_idle()
            if current_time > 5:
                for l in getattr(self, 'step_labels', []): l.config(fg=Theme.TEXT_SUB)
                if hasattr(self, 'step_labels') and len(self.step_labels) > 2: self.step_labels[2].config(fg=Theme.COLOR_ROLL)
            if current_time > 15:
                for l in getattr(self, 'step_labels', []): l.config(fg=Theme.TEXT_SUB)
                if hasattr(self, 'step_labels') and len(self.step_labels) > 3: self.step_labels[3].config(fg=Theme.COLOR_ROLL)
            time.sleep(0.05)

    def run_step_test_thread(self):
        if not self.comm or state.autotune_running or getattr(state, 'is_step_testing', False): return
        threading.Thread(target=self._step_test_task).start()

    def _step_test_task(self):
        state.is_step_testing = True
        self.btn_step_test.config(text="测试执行中...", bg=Theme.COLOR_PITCH, fg=Theme.BG_APP, state=tk.DISABLED)
        axis_val = self.tune_axis_var.get()
        try: amp = float(self.ent_step_amp.get())
        except: amp = 15.0
        state.add_log(f">>> 开始注入真实阶跃信号，目标: {amp}度")
        with state._lock:
            state.test_time_buffer.clear(); state.test_target_buffer.clear(); state.test_actual_buffer.clear()
        start_time = time.time()
        while time.time() - start_time < 0.5:
            state.target_roll = 0; state.target_pitch = 0; state.target_yaw = 0
            self._record_step_data(start_time, axis_val, 0); time.sleep(0.02)
        step_start = time.time()
        while time.time() - step_start < 2.0:
            if axis_val == "1": state.target_roll = amp
            elif axis_val == "2": state.target_pitch = amp
            elif axis_val == "4": state.target_yaw = amp
            self._record_step_data(start_time, axis_val, amp); time.sleep(0.02)
        end_start = time.time()
        while time.time() - end_start < 0.5:
            state.target_roll = 0; state.target_pitch = 0; state.target_yaw = 0
            self._record_step_data(start_time, axis_val, 0); time.sleep(0.02)
        state.is_step_testing = False
        state.add_log("<<< 阶跃测试完毕，正在计算性能指标...")
        self.root.after(0, lambda: self._calculate_and_show_metrics(amp))
        self.root.after(0, lambda: self.btn_step_test.config(text="⚡ 运行阶跃性能测试", bg="#182c25", fg=Theme.COLOR_THRUST, state=tk.NORMAL))

    def _record_step_data(self, start_time, axis_val, target):
        with state._lock:
            act = state.roll if axis_val == "1" else state.pitch if axis_val == "2" else state.yaw
            state.test_time_buffer.append(time.time() - start_time)
            state.test_target_buffer.append(target)
            state.test_actual_buffer.append(act)

    def _calculate_and_show_metrics(self, target_amp):
        times = []; actuals = []
        with state._lock:
            for t, tgt, act in zip(state.test_time_buffer, state.test_target_buffer, state.test_actual_buffer):
                if tgt > 0: times.append(t); actuals.append(act)
        if not times: return
        times = np.array(times) - times[0]; actuals = np.array(actuals)
        steady_val = np.mean(actuals[-20:]) if len(actuals) >= 20 else actuals[-1]
        peak_val = np.max(actuals); peak_idx = np.argmax(actuals); peak_time = times[peak_idx]
        overshoot = max(0, (peak_val - steady_val) / steady_val * 100) if steady_val > 1.0 else 0.0
        steady_error = abs(steady_val - target_amp)
        try:
            t10 = times[np.where(actuals >= 0.1 * steady_val)[0][0]]
            t90 = times[np.where(actuals >= 0.9 * steady_val)[0][0]]
            rise_time = (t90 - t10) * 1000
        except: rise_time = 0
        settling_time = 0
        for i in range(len(actuals)-1, -1, -1):
            if abs(actuals[i] - steady_val) > 0.05 * abs(steady_val):
                if i + 1 < len(times): settling_time = times[i+1] * 1000
                break
        metrics = {
            "超调量": f"{overshoot:.1f}", "上升时间": f"{rise_time:.0f}", "稳态误差": f"{steady_error:.2f}",
            "峰值时间": f"{peak_time*1000:.0f}", "调节时间": f"{settling_time:.0f}", "稳态值": f"{steady_val:.2f}"
        }
        for k, v in metrics.items():
            if k in getattr(self, 'perf_labels', {}): self.perf_labels[k].config(text=v)
        self.line_target.set_data(state.test_time_buffer, state.test_target_buffer)
        self.line_before.set_data([], [])
        self.line_after.set_data(state.test_time_buffer, state.test_actual_buffer)
        times_all = state.test_time_buffer
        if times_all:
            self.ax_tune.set_xlim(times_all[0], times_all[-1])
            min_y = min(state.test_actual_buffer) - 5
            max_y = max(max(state.test_actual_buffer), target_amp) + 5
            self.ax_tune.set_ylim(min_y, max_y)
        self.canvas_tune.draw_idle()

    def reset_autotune_ui(self):
        self.btn_autotune.config(text="▷ 开始自动整定", bg=Theme.COLOR_ROLL, fg=Theme.BG_APP, state=tk.NORMAL)
        for l in getattr(self, 'step_labels', []): l.config(fg=Theme.TEXT_SUB)
        if hasattr(self, 'step_labels') and self.step_labels: self.step_labels[0].config(fg=Theme.COLOR_ROLL)

    def update_param_table(self):
        with state._lock:
            for pre in ['Roll', 'Pitch', 'Yaw']:
                for p in ['P', 'I', 'D']:
                    ui_key = f"{pre}_{p}"
                    o_v = state.old_params.get(ui_key, 0.0); n_v = state.ui_params.get(ui_key, 0.0)
                    if hasattr(self, 'comp_labels') and pre in self.comp_labels:
                        self.comp_labels[pre][p]['old'].config(text=f"{o_v:.3f}")
                        if abs(n_v - o_v) > 0.0001:
                            self.comp_labels[pre][p]['new'].config(text=f"{n_v:.3f}")
                        else:
                            self.comp_labels[pre][p]['new'].config(text="-")

    def refresh_pids(self):
        self.root.focus_set()  # 【新增】强行移走输入框焦点
        if self.comm: self.comm.request_pid_params()

    def write_manual_params(self):
        self.root.focus_set()  # 【新增】强行移走输入框焦点
        if not self.comm: return
        for ui_key, ent in getattr(self, 'manual_entries', {}).items():
            try:
                val = float(ent.get())
                real_param_name = state.real_param_map.get(ui_key)
                if real_param_name:
                    self.comm.set_param_async(real_param_name, val)
            except ValueError: pass
        state.add_log(">>> 已向飞控下发覆盖参数！")
        self.root.after(500, self.refresh_pids)

    def restore_initial_params(self):
        self.root.focus_set()  # 【新增】强行移走输入框焦点
        if not self.comm or not state.initial_params_snapshot: 
            messagebox.showinfo("提示", "目前没有可用的参数快照备份！请先连接飞控获取参数。")
            return
            
        if messagebox.askyesno("安全回滚", "确定要将这 9 个 PID 参数强行覆盖回【本次刚连接时】的状态吗？\n\n注意：这将彻底撤销你今天所有的手动修改和自整定结果。"):
            for ui_key, v in state.initial_params_snapshot.items():
                real_param_name = state.real_param_map.get(ui_key)
                if real_param_name:
                    self.comm.set_param_async(real_param_name, v)
            state.add_log(">>> 【警告】已强行将参数回滚至本次连接时的安全初始状态！")
            self.root.after(500, self.refresh_pids)


    def update_loop(self):
        td = time.time() - state.last_packet_time
        if state.connected and td < 3.0:
            self.cv_dot.itemconfig(self.dot_id, fill=Theme.COLOR_THRUST)
            if hasattr(self, 'lbl_big_link'): self.lbl_big_link.config(text="连接正常 (LINK OK)", fg=Theme.COLOR_THRUST)
            if hasattr(self, 'lbl_latency_top'): self.lbl_latency_top.config(text=f"{td*1000:.1f} ms")
            if hasattr(self, 'lbl_lat_bot'): self.lbl_lat_bot.config(text=f"{td*1000:.1f} ms")
            if hasattr(self, 'lbl_model'): self.lbl_model.config(text="PX4/Ardupilot") 
            if hasattr(self, 'lbl_ip_info'): self.lbl_ip_info.config(text=f"{self.ent_ip.get()}:{self.ent_port.get()}")
            if hasattr(self, 'lbl_sysid'): self.lbl_sysid.config(text=f"{state.target_system}")
            if hasattr(self, 'lbl_bat'): self.lbl_bat.config(text=f"{state.battery_voltage:.2f} V")
            if hasattr(self, 'lbl_v_roll'): self.lbl_v_roll.config(text=f"{state.roll:.1f}")
            if hasattr(self, 'lbl_v_pitch'): self.lbl_v_pitch.config(text=f"{state.pitch:.1f}")
            if hasattr(self, 'lbl_v_yaw'): self.lbl_v_yaw.config(text=f"{state.yaw:.1f}")
            
            with state._lock:
                row = [
                    f"{time.time():.4f}", round(state.roll, 1), round(state.pitch, 1), round(state.yaw, 1), 
                    round(state.target_thrust, 3), state.servo_outputs[0], state.servo_outputs[1], state.servo_outputs[2], state.servo_outputs[3]
                ]
                state.telemetry_data.append(row)
            
            for i, v in enumerate([state.roll, state.pitch, state.yaw, state.target_thrust]):
                self.y_data[i] = np.roll(self.y_data[i], -1); self.y_data[i][-1] = v
                self.line_status[i].set_ydata(self.y_data[i])
            self.plot_counter += 1
            if self.plot_counter >= 2:
                self.canvas_status.draw_idle()
                self.plot_counter = 0
            
            if self.active_tab == 'console': 
                self.draw_attitude(self.cv_att, state.roll, state.pitch)

            for i, (cv, rect_id, lbl) in enumerate(self.pwm_bars):
                val = state.servo_outputs[i]
                w = cv.winfo_width()
                if w > 1: 
                    fill_w = max(0, min(w, (val - 1000) / 1000.0 * w))
                    cv.coords(rect_id, 0, 0, fill_w, 12) 
                lbl.config(text=str(val))

            if state.autotune_running:
                msg = state.autotune_status_msg.lower()
                if "done" in msg or "success" in msg:
                    state.autotune_running = False; self.reset_autotune_ui()
                    for l in getattr(self, 'step_labels', []): l.config(fg=Theme.TEXT_SUB)
                    if hasattr(self, 'step_labels') and self.step_labels: self.step_labels[-1].config(fg=Theme.COLOR_THRUST) 
                    self.refresh_pids()
                    self.root.after(1000, self.update_param_table)
                    state.autotune_status_msg = "Ready"
                elif "fail" in msg or "timeout" in msg or "abort" in msg:
                    state.autotune_running = False; self.reset_autotune_ui(); self.comm.send_autotune_cmd(False, 0) 
                    messagebox.showerror("整定失败", f"飞控底层终止了整定！\n原因可能为: 未装桨叶、姿态异常或无物理响应。\n飞控原话: {state.autotune_status_msg}")
                    state.autotune_status_msg = "Ready"
                elif "identify" in msg or "辨识" in msg:
                    for l in getattr(self, 'step_labels', []): l.config(fg=Theme.TEXT_SUB)
                    if hasattr(self, 'step_labels') and len(self.step_labels) > 3: self.step_labels[3].config(fg=Theme.COLOR_ROLL)

            with state._lock:
                for ui_key, v in state.param_updates.copy().items():
                    if hasattr(self, 'manual_entries') and ui_key in self.manual_entries:
                        ent = self.manual_entries[ui_key]
                        if self.root.focus_get() != ent:
                            ent.delete(0, tk.END)
                            ent.insert(0, f"{v:.4f}") 
                state.param_updates.clear()
        else:
            self.cv_dot.itemconfig(self.dot_id, fill=Theme.COLOR_DANGER)
            if hasattr(self, 'lbl_big_link'): self.lbl_big_link.config(text="离线 (OFFLINE)", fg=Theme.COLOR_DANGER)
            if hasattr(self, 'lbl_latency_top'): self.lbl_latency_top.config(text="-- ms")
            if hasattr(self, 'lbl_lat_bot'): self.lbl_lat_bot.config(text="--")
            if hasattr(self, 'lbl_model'): self.lbl_model.config(text="--")

        while state.log_queue:
            self.log_text.insert(tk.END, state.log_queue.pop(0) + "\n")
            self.log_text.see(tk.END)
        lines = int(float(self.log_text.index('end-1c')))
        if lines > 300:
            self.log_text.delete('1.0', f"{lines - 300 + 1}.0")

        self.root.after(50, self.update_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneApp(root)
    root.mainloop()
