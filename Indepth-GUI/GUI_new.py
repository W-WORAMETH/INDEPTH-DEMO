import tkinter as tk
from tkinter import messagebox, ttk
import threading
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import re
import signal

script_dir = os.path.dirname(__file__)
image_path = os.path.join(script_dir, "image/logo.png")

# --- CUSTOM WIDGETS ---
class RoundedFrame(tk.Canvas):
    def __init__(self, parent, bg_color, corner_radius=15, padding=10):
        super().__init__(parent, borderwidth=0, highlightthickness=0, bg=parent['bg'])
        self.corner_radius = corner_radius
        self.bg_color = bg_color
        self.padding = padding
        
        self.inner_frame = tk.Frame(self, bg=bg_color)
        self.window_item = self.create_window(padding, padding, window=self.inner_frame, anchor="nw")
        
        self.inner_frame.bind("<Configure>", self._on_frame_configure)
        self.bind("<Configure>", self._on_canvas_configure)

    def _on_frame_configure(self, event):
        canvas_height = event.height + 2 * self.padding
        if self.winfo_height() != canvas_height:
             self.configure(height=canvas_height)
        self._draw_background(self.winfo_width(), canvas_height)

    def _on_canvas_configure(self, event):
        frame_width = event.width - 2 * self.padding
        self.itemconfigure(self.window_item, width=frame_width)
        self._draw_background(event.width, event.height)

    def _draw_background(self, w, h):
        self.delete("bg")
        if w > 0 and h > 0:
            self._round_rectangle(0, 0, w, h, self.corner_radius, fill=self.bg_color, tags="bg")
            self.tag_lower("bg")

    def _round_rectangle(self, x1, y1, x2, y2, r, **kwargs):
        points = (x1+r, y1, x1+r, y1, x2-r, y1, x2-r, y1, x2, y1, x2, y1+r, x2, y1+r, x2, y2-r, x2, y2-r, x2, y2, x2-r, y2, x2-r, y2, x1+r, y2, x1+r, y2, x1, y2, x1, y2-r, x1, y2-r, x1, y1+r, x1, y1+r, x1, y1)
        return self.create_polygon(points, **kwargs, smooth=True)

# --- ROS 2 NODE ---
class TopicMonitorNode(Node):
    def __init__(self, update_callback, trigger_callback):
        super().__init__('topic_monitor_node')
        self.update_callback = update_callback
        self.trigger_callback = trigger_callback
        self.gui = None 

        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # --- Publishers ---
        self.motor_publisher = self.create_publisher(Int32, "/INDEPTH/MotorDrive", 10)
        self.supply_pump_publisher = self.create_publisher(Int32, "/INDEPTH/SupplyPump", 10)
        self.suction_pump_publisher = self.create_publisher(Int32, "/INDEPTH/SuctionPump", 10)

        # --- Subscribers ---
        self.raw_pos_sub = self.create_subscription(Int32, "/INDEPTH/RawPosition", self.raw_pos_callback, 10)
        self.rev_count_sub = self.create_subscription(Float32, "/INDEPTH/RevCount", self.rev_count_callback, 10)

    def set_gui(self, gui):
        self.gui = gui

    def raw_pos_callback(self, msg):
        if self.gui:
            self.gui.update_raw_pos(msg.data)

    def rev_count_callback(self, msg):
        if self.gui:
            self.gui.update_rev_count(msg.data)

    def publish_motor_speed(self, speed_value):
        msg = Int32()
        msg.data = int(speed_value)
        self.motor_publisher.publish(msg)
        print(f"Published Motor Speed: {speed_value}")

    def publish_supply_pump(self, pwm_value):
        msg = Int32()
        msg.data = int(pwm_value)
        self.supply_pump_publisher.publish(msg)
        print(f"Published Supply Pump PWM: {pwm_value}")

    def publish_suction_pump(self, pwm_value):
        msg = Int32()
        msg.data = int(pwm_value)
        self.suction_pump_publisher.publish(msg)
        print(f"Published Suction Pump PWM: {pwm_value}")


# --- MODERN GUI CLASS ---
class MonitorGUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("INDEPTH Monitoring Dashboard")
        self.root.geometry("1100x900")
        self.root.resizable(True, True)
        
        # --- THEME COLORS ---
        self.colors = {
            "bg_dark": "#212121",       # Main background 212121
            "card_bg": "#303030",       # Card background
            "text_main": "#E0E0E0",     # Primary text
            "text_sub": "#B0B0B0",      # Secondary text
            "accent_cyan": "#00E5FF",   # Motor / Highlighting
            "accent_green": "#00E676",  # Success / On
            "accent_red": "#FF1744",    # Error / Off
            "accent_orange": "#FF9100", # Pumps
            "input_bg": "#424242"       # Entry fields
        }
        
        self.root.configure(bg=self.colors["bg_dark"])
        self.setup_styles()

        self.process = None 
        self.ros2_node = None
        
        # Logic Variables
        self.motor_timer_id = 0 
        self.is_supply_on = False
        self.is_suction_on = False
        self.limit_triggered = False
        
        self.ansi_colors = {
            "30": "black", "31": "red", "32": "green", "33": "yellow",
            "34": "blue", "35": "magenta", "36": "cyan", "37": "white"
        }

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.create_widgets()

    def setup_styles(self):
        style = ttk.Style()
        style.theme_use('alt') # 'alt' usually renders a clean checkmark
        
        # General Frame
        style.configure("TFrame", background=self.colors["bg_dark"])
        style.configure("Card.TFrame", background=self.colors["card_bg"], relief="flat")
        
        # Labels
        style.configure("TLabel", background=self.colors["card_bg"], foreground=self.colors["text_main"], font=("Segoe UI", 10))
        style.configure("Title.TLabel", background=self.colors["card_bg"], foreground=self.colors["accent_cyan"], font=("Segoe UI", 14, "bold"))
        style.configure("Header.TLabel", background=self.colors["bg_dark"], foreground=self.colors["text_main"], font=("Segoe UI", 18, "bold"))
        
        # Buttons
        style.configure("TButton", background="#424242", foreground="white", borderwidth=0, focuscolor="none", font=("Segoe UI", 10, "bold"))
        style.map("TButton", background=[("active", "#616161")])
        
        style.configure("Action.TButton", background=self.colors["accent_cyan"], foreground="black")
        style.map("Action.TButton", background=[("active", "#84FFFF")])

        # Checkbutton
        style.configure("TCheckbutton", background=self.colors["card_bg"], foreground=self.colors["text_main"], font=("Segoe UI", 10))
        style.map("TCheckbutton", background=[("active", self.colors["card_bg"])], indicatorcolor=[("selected", self.colors["accent_cyan"])])

    def create_card_frame(self, parent, title):
        """Helper to create a styled card with a title"""
        # Create the Wrapper (Canvas)
        wrapper = RoundedFrame(parent, bg_color=self.colors["card_bg"], corner_radius=15, padding=15)
        wrapper.pack(fill=tk.X, pady=(0, 15)) # Pack immediately
        
        # We work with the inner frame
        card = wrapper.inner_frame
        
        # Title with cyan accent line
        header_frame = tk.Frame(card, bg=self.colors["card_bg"])
        header_frame.pack(fill=tk.X, pady=(0, 10))
        
        tk.Label(header_frame, text=title, bg=self.colors["card_bg"], fg=self.colors["accent_cyan"], 
                 font=("Segoe UI", 12, "bold")).pack(side=tk.LEFT)
        
        return card

    def create_widgets(self):
        # --- HEADER ---
        header_frame = tk.Frame(self.root, bg=self.colors["bg_dark"], pady=10)
        header_frame.pack(fill=tk.X, padx=20)
        
        if os.path.exists(image_path):
            self.image = tk.PhotoImage(file=image_path)
            # Resize if needed? For now just place it
            tk.Label(header_frame, image=self.image, bg=self.colors["bg_dark"]).pack(side=tk.LEFT, padx=10)
            
        tk.Label(header_frame, text="INDEPTH SYSTEM MONITOR", bg=self.colors["bg_dark"], fg="white", 
                 font=("Segoe UI", 24, "bold")).pack(side=tk.LEFT, padx=20)

        # --- MAIN GRID ---
        main_content = tk.Frame(self.root, bg=self.colors["bg_dark"])
        main_content.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
        
        # Configure grid weight
        # main_content.columnconfigure(0, weight=1) # Controls
        # main_content.columnconfigure(1, weight=1) # Feedback

        # === COLUMN 1 : CONTROLS ===
        col1 = tk.Frame(main_content, bg=self.colors["bg_dark"])
        col1.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))
        
        # 1. Micro-ROS Agent Card
        self.build_agent_card(col1)

        # 2. Motor Control Card
        self.build_motor_card(col1)

        # 3. Pump Control Card
        self.build_pump_card(col1)

        # === COLUMN 2 : FEEDBACK ===
        col2 = tk.Frame(main_content, bg=self.colors["bg_dark"])
        col2.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(10, 0))

        # 4. Monitoring & Limits Card
        self.build_monitoring_card(col2)

        # 5. Terminal Card
        self.build_terminal_card(col2)

    def build_agent_card(self, parent):
        card = self.create_card_frame(parent, "Connection Control")
        # card.pack removed, handled in create_card_frame
        
        self.ins_label = tk.Label(card, text="Agent Status: Not connected", bg=self.colors["card_bg"], fg=self.colors["text_sub"])
        self.ins_label.pack(pady=(0, 10),anchor=tk.W)

        btn_frame = tk.Frame(card, bg=self.colors["card_bg"])
        btn_frame.pack(fill=tk.X)

        self.start_agent_button = tk.Button(btn_frame, text="Start Wi-Fi Agent", bg="#0D47A1", fg="white", font=("Segoe UI", 10),
                                            command=self.start_ros2_agent, relief="flat", padx=10, pady=5)
        self.start_agent_button.pack(side=tk.LEFT, padx=(0, 10))

        self.start_agentserial_button = tk.Button(btn_frame, text="Start Serial Agent", bg="#0D47A1", fg="white", font=("Segoe UI", 10),
                                                  command=self.start_ros2_agent_serial, relief="flat", padx=10, pady=5)
        self.start_agentserial_button.pack(side=tk.LEFT, padx=(0, 10))

        self.stop_agent_button = tk.Button(btn_frame, text="Stop Agent", bg="#B71C1C", fg="white", font=("Segoe UI", 10),
                                           state=tk.DISABLED, command=self.stop_ros2_agent, relief="flat", padx=10, pady=5)
        self.stop_agent_button.pack(side=tk.LEFT)

    def build_motor_card(self, parent):
        card = self.create_card_frame(parent, "Motor Control")
        # card.pack removed
        
        # Slider
        tk.Label(card, text="Speed (-200 to 200)", bg=self.colors["card_bg"], fg=self.colors["text_sub"]).pack(anchor=tk.W)
        self.motor_speed_slider = tk.Scale(card, from_=-200, to=200, orient=tk.HORIZONTAL, length=400,
                                           bg=self.colors["card_bg"], fg=self.colors["text_sub"], troughcolor=self.colors["input_bg"],
                                           highlightthickness=0, borderwidth=0)
        self.motor_speed_slider.set(0)
        self.motor_speed_slider.pack(fill=tk.X, pady=(0, 10))
        
        # Presets Grid
        preset_frame = tk.Frame(card, bg=self.colors["card_bg"])
        preset_frame.pack(fill=tk.X, pady=5)
        
        presets = [-200, -100, -50, 0, 50, 100, 200]
        for val in presets:
            btn = tk.Button(preset_frame, text=str(val), width=5, bg="#424242", fg="white",
                            command=lambda v=val: self.motor_speed_slider.set(v))
            btn.pack(side=tk.LEFT, padx=2)

        # Timer
        time_frame = tk.Frame(card, bg=self.colors["card_bg"])
        time_frame.pack(fill=tk.X, pady=10)
        
        self.use_timer_var = tk.BooleanVar(value=False)
        chk = ttk.Checkbutton(time_frame, text="Auto-stop after (sec):", variable=self.use_timer_var, command=self.toggle_timer_entry)
        chk.pack(side=tk.LEFT)
        
        self.timer_duration_var = tk.StringVar(value="5.0")
        self.ent_duration = tk.Entry(time_frame, textvariable=self.timer_duration_var, width=6, bg=self.colors["input_bg"], fg="white",
                                     disabledbackground=self.colors["bg_dark"], disabledforeground="#616161", state="disabled")
        self.ent_duration.pack(side=tk.LEFT, padx=5)

        # Set Speed Button
        tk.Button(card, text="SET MOTOR SPEED", bg=self.colors["accent_cyan"], fg="black", font=("Segoe UI", 11, "bold"),
                  command=self.send_motor_command, padx=20, pady=5).pack(pady=10)

    def build_pump_card(self, parent):
        card = self.create_card_frame(parent, "Pump Control")
        # card.pack removed
        
        # Grid for Supply vs Suction
        grid_frame = tk.Frame(card, bg=self.colors["card_bg"])
        grid_frame.pack(fill=tk.X)
        grid_frame.columnconfigure(0, weight=1)
        grid_frame.columnconfigure(1, weight=1)
        
        # -- Supply Pump --
        supply_frame = tk.Frame(grid_frame, bg=self.colors["card_bg"])
        supply_frame.grid(row=0, column=0, sticky="ew", padx=5)
        
        tk.Label(supply_frame, text="Supply Pump", bg=self.colors["card_bg"], fg=self.colors["accent_orange"], font=("Segoe UI", 10, "bold")).pack()
        
        # PWM Input
        in_frame = tk.Frame(supply_frame, bg=self.colors["card_bg"])
        in_frame.pack(pady=5)
        tk.Label(in_frame, text="PWM:", bg=self.colors["card_bg"], fg="#BDBDBD").pack(side=tk.LEFT)
        self.supply_pwm_var = tk.StringVar(value="50")
        self.ent_supply_pwm = tk.Entry(in_frame, textvariable=self.supply_pwm_var, width=4, bg=self.colors["input_bg"], fg="white", justify="center")
        self.ent_supply_pwm.pack(side=tk.LEFT, padx=5)
        self.ent_supply_pwm.bind('<Return>', lambda e: self.update_pump_if_on("supply"))

        self.btn_supply_toggle = tk.Button(supply_frame, text="OFF", bg=self.colors["accent_red"], fg="white", width=8,
                                           command=self.toggle_supply_pump)
        self.btn_supply_toggle.pack(pady=5)

        # -- Suction Pump --
        suction_frame = tk.Frame(grid_frame, bg=self.colors["card_bg"])
        suction_frame.grid(row=0, column=1, sticky="ew", padx=5)
        
        tk.Label(suction_frame, text="Suction Pump", bg=self.colors["card_bg"], fg=self.colors["accent_orange"], font=("Segoe UI", 10, "bold")).pack()
        
        # PWM Input
        in_frame2 = tk.Frame(suction_frame, bg=self.colors["card_bg"])
        in_frame2.pack(pady=5)
        tk.Label(in_frame2, text="PWM:", bg=self.colors["card_bg"], fg="#BDBDBD").pack(side=tk.LEFT)
        self.suction_pwm_var = tk.StringVar(value="50")
        self.ent_suction_pwm = tk.Entry(in_frame2, textvariable=self.suction_pwm_var, width=4, bg=self.colors["input_bg"], fg="white", justify="center")
        self.ent_suction_pwm.pack(side=tk.LEFT, padx=5)
        self.ent_suction_pwm.bind('<Return>', lambda e: self.update_pump_if_on("suction"))

        self.btn_suction_toggle = tk.Button(suction_frame, text="OFF", bg=self.colors["accent_red"], fg="white", width=8,
                                           command=self.toggle_suction_pump)
        self.btn_suction_toggle.pack(pady=5)

    def build_monitoring_card(self, parent):
        card = self.create_card_frame(parent, "Live Monitoring")
        # card.pack removed
        
        # -- Live Values --
        val_frame = tk.Frame(card, bg=self.colors["card_bg"])
        val_frame.pack(fill=tk.X, pady=10)
        
        # Raw Pos
        f1 = tk.Frame(val_frame, bg=self.colors["card_bg"])
        f1.pack(side=tk.LEFT, padx=20)
        tk.Label(f1, text="Raw Position", bg=self.colors["card_bg"], fg=self.colors["text_sub"], font=("Segoe UI", 10)).pack()
        self.lbl_raw_pos = tk.Label(f1, text="--", bg=self.colors["card_bg"], fg=self.colors["accent_cyan"], font=("Consolas", 20, "bold"))
        self.lbl_raw_pos.pack()

        # Rev Count
        f2 = tk.Frame(val_frame, bg=self.colors["card_bg"])
        f2.pack(side=tk.LEFT, padx=20)
        tk.Label(f2, text="Rev Count", bg=self.colors["card_bg"], fg=self.colors["text_sub"], font=("Segoe UI", 10)).pack()
        self.lbl_rev_count = tk.Label(f2, text="--", bg=self.colors["card_bg"], fg=self.colors["accent_green"], font=("Consolas", 20, "bold"))
        self.lbl_rev_count.pack()
        
        # -- LED Status --
        f3 = tk.Frame(val_frame, bg=self.colors["card_bg"])
        f3.pack(side=tk.RIGHT, padx=20)
        tk.Label(f3, text="Status", bg=self.colors["card_bg"], fg=self.colors["text_sub"], font=("Segoe UI", 10)).pack()
        self.led_canvas = tk.Canvas(f3, width=30, height=30, bg=self.colors["card_bg"], highlightthickness=0)
        self.led_canvas.pack()
        self.led_circle = self.led_canvas.create_oval(5, 5, 25, 25, fill="green", outline="")
        
        # -- Limits Control --
        tk.Frame(card, height=1, bg="#616161").pack(fill=tk.X, pady=15) # Separator
        tk.Label(card, text="Safety Limits", bg=self.colors["card_bg"], fg=self.colors["text_main"], font=("Segoe UI", 11, "bold")).pack(anchor=tk.W)

        # Max Limit
        max_frame = tk.Frame(card, bg=self.colors["card_bg"])
        max_frame.pack(fill=tk.X, pady=5)
        self.use_upper_limit_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(max_frame, text="Enable Max Limit:", variable=self.use_upper_limit_var).pack(side=tk.LEFT)
        self.upper_limit_var = tk.StringVar(value="5000")
        tk.Entry(max_frame, textvariable=self.upper_limit_var, width=8, bg=self.colors["input_bg"], fg="white").pack(side=tk.LEFT, padx=10)
        
        # Min Limit
        min_frame = tk.Frame(card, bg=self.colors["card_bg"])
        min_frame.pack(fill=tk.X, pady=5)
        self.use_lower_limit_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(min_frame, text="Enable Min Limit:", variable=self.use_lower_limit_var).pack(side=tk.LEFT)
        self.lower_limit_var = tk.StringVar(value="0")
        tk.Entry(min_frame, textvariable=self.lower_limit_var, width=8, bg=self.colors["input_bg"], fg="white").pack(side=tk.LEFT, padx=10)
        
        # Reset Button
        tk.Button(card, text="RESET LIMIT STATE", bg="#424242", fg="white", font=("Segoe UI", 9),
                  command=self.reset_limit_state).pack(pady=10, fill=tk.X)

    def build_terminal_card(self, parent):
        card = self.create_card_frame(parent, "System Log")
        # Ensure terminal card expands
        card.master.pack_configure(fill=tk.BOTH, expand=True)
        
        self.terminal_output = tk.Text(card, bg="black", fg="#E0E0E0", font=("Consolas", 10), height=15)
        self.terminal_output.pack(fill=tk.BOTH, expand=True)
        
        # Scrollbar
        scrollbar = ttk.Scrollbar(self.terminal_output, command=self.terminal_output.yview)
        # scrollbar.pack(side=tk.RIGHT, fill=tk.Y) # Tkinter Text scrollbar packing is tricky inside Text widget usually
        self.terminal_output.config(yscrollcommand=scrollbar.set)


    # --- LOGIC METHODS ---
    def update_raw_pos(self, value):
        self.root.after(0, lambda: self.lbl_raw_pos.config(text=str(value)))

    def update_rev_count(self, value):
        self.root.after(0, lambda: self.lbl_rev_count.config(text=f"{value:.2f}"))
        
        # Limit Logic
        if not self.limit_triggered:
            # Check Upper
            if self.use_upper_limit_var.get():
                try:
                    limit = float(self.upper_limit_var.get())
                    if value >= limit and self.motor_speed_slider.get() != 0:
                        self.stop_motor_due_to_limit(limit, "Max")
                except ValueError: pass
            
            # Check Lower
            if not self.limit_triggered and self.use_lower_limit_var.get():
                try:
                    limit = float(self.lower_limit_var.get())
                    if value <= limit and self.motor_speed_slider.get() != 0:
                        self.stop_motor_due_to_limit(limit, "Min")
                except ValueError: pass

    def stop_motor_due_to_limit(self, limit, limit_type):
        if self.ros2_node:
            self.limit_triggered = True
            self.ros2_node.publish_motor_speed(0)
            self.root.after(0, lambda: self.handle_limit_stop_gui(limit, limit_type))

    def handle_limit_stop_gui(self, limit, limit_type):
        self.motor_speed_slider.set(0)
        self.log_to_terminal(f"[Limit] Stopped! {limit_type} limit {limit} reached.", color="red")
        self.led_canvas.itemconfig(self.led_circle, fill="red")
        messagebox.showwarning("Limit Reached", f"{limit_type} Limit ({limit}) Reached!")

    def reset_limit_state(self):
        self.limit_triggered = False
        self.led_canvas.itemconfig(self.led_circle, fill="green")
        self.log_to_terminal("[Limit] State reset. Motor enabled.", color="green")

    def log_to_terminal(self, msg, color="white"):
        self.terminal_output.insert(tk.END, f"\n{msg}", (color,))
        self.terminal_output.tag_config(color, foreground=color if color != "white" else "#E0E0E0")
        self.terminal_output.see(tk.END)
    
    def read_terminal_output(self):
        ansi_escape = re.compile(r'\x1B\[(\d+)m')
        if self.process:
            for line in iter(self.process.stdout.readline, ""):
                self.terminal_output.insert(tk.END, "\n")
                matches = ansi_escape.findall(line)
                color = "white"
                for match in matches:
                    if match in self.ansi_colors:
                        color = self.ansi_colors[match]
                
                cleaned_line = ansi_escape.sub("", line)
                # Remove newline from cleaned_line if double spacing occurs
                cleaned_line = cleaned_line.rstrip() 
                
                self.terminal_output.insert(tk.END, cleaned_line, (color,))
                self.terminal_output.tag_config(color, foreground=color if color != "white" else "#E0E0E0")
                self.terminal_output.see(tk.END)

    def toggle_timer_entry(self):
        if self.use_timer_var.get():
            self.ent_duration.config(state="normal")
        else:
            self.ent_duration.config(state="disabled")

    def send_motor_command(self):
        if self.ros2_node:
            value = self.motor_speed_slider.get()
            self.motor_timer_id += 1
            
            # Publish
            self.ros2_node.publish_motor_speed(value)
            
            # Log
            log_msg = f"\n[GUI] Sent Motor Speed: {value}"
            if self.use_timer_var.get():
                try:
                    duration = float(self.timer_duration_var.get())
                    if duration > 0:
                        log_msg += f" (Auto-stop in {duration}s)"
                        threading.Thread(target=self.motor_stop_thread, 
                                         args=(duration, self.motor_timer_id), daemon=True).start()
                except ValueError:
                    messagebox.showerror("Error", "Invalid duration")
            
            # self.log_to_terminal(log_msg) # Need to implement terminal
            print(log_msg)

    def motor_stop_thread(self, duration, current_timer_id):
        time.sleep(duration)
        if self.motor_timer_id == current_timer_id:
            if self.ros2_node:
                self.ros2_node.publish_motor_speed(0)
                self.root.after(0, self.update_after_stop)

    def update_after_stop(self):
        self.motor_speed_slider.set(0)
        print("\n[Timer] Auto-stopped.")

    def validate_pwm(self, var):
        try:
            val = int(var.get())
            if val < 0: val = 0
            if val > 100: val = 100
            return val
        except ValueError:
            return 0

    def toggle_supply_pump(self):
        if not self.ros2_node: return
        if not self.is_supply_on:
            # Turn ON
            pwm = self.validate_pwm(self.supply_pwm_var)
            self.ros2_node.publish_supply_pump(pwm)
            self.btn_supply_toggle.config(text="ON", bg=self.colors["accent_green"])
            self.is_supply_on = True
        else:
            # Turn OFF
            self.ros2_node.publish_supply_pump(0)
            self.btn_supply_toggle.config(text="OFF", bg=self.colors["accent_red"])
            self.is_supply_on = False

    def toggle_suction_pump(self):
        if not self.ros2_node: return
        if not self.is_suction_on:
            # Turn ON
            pwm = self.validate_pwm(self.suction_pwm_var)
            self.ros2_node.publish_suction_pump(pwm)
            self.btn_suction_toggle.config(text="ON", bg=self.colors["accent_green"])
            self.is_suction_on = True
        else:
            # Turn OFF
            self.ros2_node.publish_suction_pump(0)
            self.btn_suction_toggle.config(text="OFF", bg=self.colors["accent_red"])
            self.is_suction_on = False

    def update_pump_if_on(self, pump_type):
        if not self.ros2_node: return
        if pump_type == "supply" and self.is_supply_on:
            pwm = self.validate_pwm(self.supply_pwm_var)
            self.ros2_node.publish_supply_pump(pwm)
        elif pump_type == "suction" and self.is_suction_on:
            pwm = self.validate_pwm(self.suction_pwm_var)
            self.ros2_node.publish_suction_pump(pwm)

    def start_ros2_agent(self):
        if self.process is None:
            self.process = subprocess.Popen(
                ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "udp4", "--port", "8888"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
            )
            self._update_agent_buttons(started=True, mode="Wi-Fi")
            threading.Thread(target=self.read_terminal_output, daemon=True).start()

    def start_ros2_agent_serial(self):
        if self.process is None:
            self.process = subprocess.Popen(
                ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev", "/dev/ttyUSB0"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
            )
            self._update_agent_buttons(started=True, mode="Serial")
            threading.Thread(target=self.read_terminal_output, daemon=True).start()

    def _update_agent_buttons(self, started, mode=""):
        if started:
            self.start_agent_button.config(state="disabled", bg="#1A237E")
            self.start_agentserial_button.config(state="disabled", bg="#1A237E")
            self.stop_agent_button.config(state="normal", bg="#D32F2F")
            self.ins_label.config(text=f"Agent Status: Connected ({mode})", fg=self.colors["accent_green"])
        else:
            self.start_agent_button.config(state="normal", bg="#0D47A1")
            self.start_agentserial_button.config(state="normal", bg="#0D47A1")
            self.stop_agent_button.config(state="disabled", bg="#B71C1C")
            self.ins_label.config(text="Agent Status: Not connected", fg=self.colors["text_sub"])

    def stop_ros2_agent(self):
        os.system("pkill -f micro_ros_agent")
        self.process = None
        self._update_agent_buttons(started=False)
    
    def on_closing(self): self.stop_ros2_agent(); self.root.destroy()
    def start(self, node): 
        self.ros2_node = node
        self.root.mainloop()

def main():
    rclpy.init()
    node = TopicMonitorNode(None, None)
    gui = MonitorGUI()
    node.set_gui(gui)

    def ros2_thread():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    threading.Thread(target=ros2_thread, daemon=True).start()
    gui.start(node)

if __name__ == "__main__":
    main()
