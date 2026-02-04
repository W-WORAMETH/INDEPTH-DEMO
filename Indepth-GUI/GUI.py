import tkinter as tk
from tkinter import messagebox
import threading
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, String, Bool, Int8, Int32, Float32
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import os
import re
import signal
import serial

script_dir = os.path.dirname(__file__)  # Directory of the script
# Ensure this path exists or handle the error if image is missing
image_path = os.path.join(script_dir, "image/logo.png")

class TopicMonitorNode(Node):
    def __init__(self, timestamp_topics, status_topics, update_callback, trigger_callback):
        super().__init__('topic_monitor_node')
        self.timestamp_subscribers = {}
        self.status_subscribers = {}
        self.update_callback = update_callback
        self.trigger_callback = trigger_callback
        self.gui = None # Reference to GUI for updates

        qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribe to TimeStamp topics
        for topic in timestamp_topics:
            self.timestamp_subscribers[topic] = self.create_subscription(
                Int64,
                topic,
                lambda msg, t=topic: self.timestamp_callback(t, msg),
                qos_profile
            )

        # Subscribe to Status topics
        for topic in status_topics:
            self.status_subscribers[topic] = self.create_subscription(
                String,
                topic,
                lambda msg, t=topic: self.status_callback(t, msg),
                10
            )

        # Publisher for trigger commands
        self.trigger_publisher = self.create_publisher(Int8, "/Visens/TriggerCommand", 10)
        self.trigger_format_publisher = self.create_publisher(Bool, "/Visens/TriggerFormat", 10)

        # --- Motor & Pump Publishers ---
        self.motor_publisher = self.create_publisher(Int32, "/INDEPTH/MotorDrive", 10)
        self.supply_pump_publisher = self.create_publisher(Int32, "/INDEPTH/SupplyPump", 10)
        self.suction_pump_publisher = self.create_publisher(Int32, "/INDEPTH/SuctionPump", 10)

        # --- NEW: Monitoring Subscribers ---
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

    # Function to publish motor speed
    def publish_motor_speed(self, speed_value):
        msg = Int32()
        msg.data = int(speed_value)
        self.motor_publisher.publish(msg)
        print(f"Published Motor Speed: {speed_value}")

    # --- NEW: Pump Publishers ---
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

    def publish_trigger(self, value):
        msg = Int8()
        msg.data = value
        self.trigger_publisher.publish(msg)

        if self.trigger_callback:
            self.trigger_callback(value)

class MonitorGUI:
    def __init__(self, timestamp_topics, status_topics):
        self.timestamp_topics = timestamp_topics
        self.status_topics = status_topics
        self.root = tk.Tk()
        self.root.title("VISENS Sensor Monitoring")

        # Fix window size and make it unchangeable
        self.root.geometry("1000x1100")
        self.root.resizable(False, False)
        self.process = None  # For ROS2 micro-ROS Agent
        self.ros2_node = None # Initialize ROS2 node holder
        
        # --- NEW: Timer ID for managing thread safety ---
        self.motor_timer_id = 0 
        
        # --- NEW: Pump State Tracking ---
        self.is_supply_on = False
        self.is_suction_on = False
        
        # --- NEW: Rev Limit Variable ---
        self.limit_triggered = False

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.create_widgets()

    def create_widgets(self):
        
        # Add logo image
        if os.path.exists(image_path):
            self.image = tk.PhotoImage(file=image_path)
            self.image_label = tk.Label(self.root, image=self.image)
            self.image_label.pack(pady=0)

        # --- SECTION : MOTOR CONTROL ---
        frame_motor = tk.Frame(self.root, bd=2, relief=tk.GROOVE)
        frame_motor.pack(padx=20, pady=20, fill=tk.X)

        lbl_motor_title = tk.Label(frame_motor, text="Motor Drive Control", font=("Arial", 12, "bold"))
        lbl_motor_title.pack(pady=(10, 5))

        # Slider container
        slider_container = tk.Frame(frame_motor)
        slider_container.pack(fill=tk.X, padx=50)

        # Slide bar (-200 to 200)
        self.motor_speed_slider = tk.Scale(
            slider_container, 
            from_=-200, 
            to=200, 
            orient=tk.HORIZONTAL,
            length=600,
            tickinterval=50, 
            label="Speed Value"
        )
        self.motor_speed_slider.set(0) # Default to 0
        self.motor_speed_slider.pack(fill=tk.X)

        # Preset Buttons
        frame_presets = tk.Frame(frame_motor)
        frame_presets.pack(pady=5)

        btn_min = tk.Button(frame_presets, text="-200", width=6, command=lambda: self.motor_speed_slider.set(-200))
        btn_min.pack(side=tk.LEFT, padx=10)

        btn_m150 = tk.Button(frame_presets, text="-150", width=6, command=lambda: self.motor_speed_slider.set(-150))
        btn_m150.pack(side=tk.LEFT, padx=10)

        btn_m100 = tk.Button(frame_presets, text="-100", width=6, command=lambda: self.motor_speed_slider.set(-100))
        btn_m100.pack(side=tk.LEFT, padx=10)

        btn_m50 = tk.Button(frame_presets, text="-50", width=6, command=lambda: self.motor_speed_slider.set(-50))
        btn_m50.pack(side=tk.LEFT, padx=10)

        btn_zero = tk.Button(frame_presets, text="0", width=6, command=lambda: self.motor_speed_slider.set(0))
        btn_zero.pack(side=tk.LEFT, padx=10)

        btn_50 = tk.Button(frame_presets, text="50", width=6, command=lambda: self.motor_speed_slider.set(50))
        btn_50.pack(side=tk.LEFT, padx=10)

        btn_100 = tk.Button(frame_presets, text="100", width=6, command=lambda: self.motor_speed_slider.set(100))
        btn_100.pack(side=tk.LEFT, padx=10)

        btn_150 = tk.Button(frame_presets, text="150", width=6, command=lambda: self.motor_speed_slider.set(150))
        btn_150.pack(side=tk.LEFT, padx=10)

        btn_max = tk.Button(frame_presets, text="200", width=6, command=lambda: self.motor_speed_slider.set(200))
        btn_max.pack(side=tk.LEFT, padx=10)

        # --- Timing Control Section ---
        frame_timing = tk.Frame(frame_motor)
        frame_timing.pack(pady=5)

        # Variable to check if we should use timer (Default False = Continuous)
        self.use_timer_var = tk.BooleanVar(value=False)
        
        self.chk_timer = tk.Checkbutton(
            frame_timing, 
            text="Auto-stop after (sec):", 
            variable=self.use_timer_var,
            command=self.toggle_timer_entry
        )
        self.chk_timer.pack(side=tk.LEFT)

        # Entry for duration (Default 5.0 seconds)
        self.timer_duration_var = tk.StringVar(value="5.0")
        self.ent_duration = tk.Entry(
            frame_timing, 
            textvariable=self.timer_duration_var, 
            width=5, 
            state=tk.DISABLED, # Disabled by default
            justify='center'
        )
        self.ent_duration.pack(side=tk.LEFT, padx=5)

        # Send Button
        self.btn_send_motor = tk.Button(
            frame_motor,
            text="Set Motor Speed",
            command=self.send_motor_command,
            bg="darkorange",
            fg="white",
            font=("Arial", 10, "bold"),
            width=20
        )
        self.btn_send_motor.pack(pady=10)
        
        # --- NEW SECTION: PUMP CONTROL ---
        # Separator line
        tk.Frame(frame_motor, height=2, bd=1, relief=tk.SUNKEN).pack(fill=tk.X, padx=10, pady=10)

        frame_pumps = tk.Frame(frame_motor)
        frame_pumps.pack(fill=tk.X, padx=10, pady=5)
        
        # Configure grid columns to be equal width
        frame_pumps.columnconfigure(0, weight=1)
        frame_pumps.columnconfigure(1, weight=1)

        # -- Left Column: Supply Pump --
        frame_supply = tk.Frame(frame_pumps)
        frame_supply.grid(row=0, column=0, sticky="ew", padx=10)

        tk.Label(frame_supply, text="Supply Pump", font=("Arial", 10, "bold")).pack()
        
        frame_supply_input = tk.Frame(frame_supply)
        frame_supply_input.pack(pady=5)
        
        tk.Label(frame_supply_input, text="PWM (0-100):").pack(side=tk.LEFT)
        self.supply_pwm_var = tk.StringVar(value="50")
        self.ent_supply_pwm = tk.Entry(frame_supply_input, textvariable=self.supply_pwm_var, width=5, justify='center')
        self.ent_supply_pwm.pack(side=tk.LEFT, padx=5)
        # Bind Enter key to update if ON
        self.ent_supply_pwm.bind('<Return>', lambda event: self.update_pump_if_on("supply"))

        self.btn_supply_toggle = tk.Button(
            frame_supply,
            text="OFF",
            bg="red",
            fg="white",
            width=10,
            command=self.toggle_supply_pump
        )
        self.btn_supply_toggle.pack(pady=5)

        # -- Right Column: Suction Pump --
        frame_suction = tk.Frame(frame_pumps)
        frame_suction.grid(row=0, column=1, sticky="ew", padx=10)

        tk.Label(frame_suction, text="Suction Pump", font=("Arial", 10, "bold")).pack()

        frame_suction_input = tk.Frame(frame_suction)
        frame_suction_input.pack(pady=5)

        tk.Label(frame_suction_input, text="PWM (0-100):").pack(side=tk.LEFT)
        self.suction_pwm_var = tk.StringVar(value="50")
        self.ent_suction_pwm = tk.Entry(frame_suction_input, textvariable=self.suction_pwm_var, width=5, justify='center')
        self.ent_suction_pwm.pack(side=tk.LEFT, padx=5)
        # Bind Enter key to update if ON
        self.ent_suction_pwm.bind('<Return>', lambda event: self.update_pump_if_on("suction"))

        self.btn_suction_toggle = tk.Button(
            frame_suction,
            text="OFF",
            bg="red",
            fg="white",
            width=10,
            command=self.toggle_suction_pump
        )
        self.btn_suction_toggle.pack(pady=5)
        
        # --- NEW SECTION: MOTOR MONITORING ---
        tk.Frame(frame_motor, height=2, bd=1, relief=tk.SUNKEN).pack(fill=tk.X, padx=10, pady=10)
        
        frame_monitor = tk.Frame(frame_motor)
        frame_monitor.pack(fill=tk.X, padx=10, pady=5)
        
        tk.Label(frame_monitor, text="Motor Feedback", font=("Arial", 10, "bold")).pack()
        
        frame_monitor_data = tk.Frame(frame_monitor)
        frame_monitor_data.pack(pady=5)
        
        # Raw Position Label
        self.lbl_raw_pos = tk.Label(frame_monitor_data, text="Raw Pos: --", font=("Courier", 12, "bold"), fg="cyan", bg="black", width=15)
        self.lbl_raw_pos.pack(side=tk.LEFT, padx=10)
        
        self.lbl_rev_count = tk.Label(frame_monitor_data, text="Rev: --", font=("Courier", 12, "bold"), fg="lime", bg="black", width=15)
        self.lbl_rev_count.pack(side=tk.LEFT, padx=10)
        
        # --- NEW SECTION: REV LIMIT CONTROL (UPPER & LOWER) ---
        frame_limit = tk.Frame(frame_monitor)
        frame_limit.pack(pady=5)
        
        # Upper Limit (Max)
        frame_upper = tk.Frame(frame_limit)
        frame_upper.pack(anchor=tk.W)
        self.use_upper_limit_var = tk.BooleanVar(value=True)
        tk.Checkbutton(frame_upper, text="Enable Max", variable=self.use_upper_limit_var).pack(side=tk.LEFT)
        tk.Label(frame_upper, text="Limit:").pack(side=tk.LEFT, padx=(5, 2))
        self.upper_limit_var = tk.StringVar(value="5000")
        self.ent_upper_limit = tk.Entry(frame_upper, textvariable=self.upper_limit_var, width=8, justify='center')
        self.ent_upper_limit.pack(side=tk.LEFT)

        # Lower Limit (Min)
        frame_lower = tk.Frame(frame_limit)
        frame_lower.pack(anchor=tk.W)
        self.use_lower_limit_var = tk.BooleanVar(value=True)
        tk.Checkbutton(frame_lower, text="Enable Min", variable=self.use_lower_limit_var).pack(side=tk.LEFT)
        tk.Label(frame_lower, text="Limit:").pack(side=tk.LEFT, padx=(5, 2))
        self.lower_limit_var = tk.StringVar(value="0")
        self.ent_lower_limit = tk.Entry(frame_lower, textvariable=self.lower_limit_var, width=8, justify='center')
        self.ent_lower_limit.pack(side=tk.LEFT)
        
        # Reset Limit State Button
        self.btn_reset_limit = tk.Button(frame_limit, text="Reset Limit State", command=self.reset_limit_state, font=("Arial", 8))
        self.btn_reset_limit.pack(pady=5)
        # ---------------------------------

        frame_agent = tk.Frame(self.root)
        frame_agent.pack(padx=10, pady=10)

        # Terminal Output Box
        self.terminal_output = tk.Text(frame_agent, wrap=tk.WORD, height=8, width=95, bg="black", fg="white")
        self.terminal_output.pack(padx=10, pady=1)

        # Define ANSI color mappings
        self.ansi_colors = {
            "30": "black", "31": "red", "32": "green", "33": "yellow",
            "34": "blue", "35": "magenta", "36": "cyan", "37": "white"
        }

        frame_agent_button = tk.Frame(self.root)
        frame_agent_button.pack(padx=10, pady=2, side=tk.TOP)
        
        # Start Button
        self.ins_label = tk.Label(self.root, text="Micro-ros agent is not running [please start agent within 5 second after power the sensor modules]", width=100, anchor='c', font=("Arial", 10, "bold"))
        self.ins_label.pack(pady=1)
        
        self.start_agent_button = tk.Button(
            frame_agent_button, 
            text="Start Wi-Fi micro-ROS Agent", 
            command=self.start_ros2_agent,
            bg="dodgerblue3", 
            fg="white",
            activebackground="dodgerblue4",
            activeforeground="white",
            disabledforeground="gray",
        )
        self.start_agent_button.pack(pady=1, side=tk.LEFT)

        self.start_agentserial_button = tk.Button(
            frame_agent_button, 
            text="Start Serial micro-ROS Agent", 
            command=self.start_ros2_agent_serial,
            bg="dodgerblue3", 
            fg="white",
            activebackground="dodgerblue4",
            activeforeground="white",
            disabledforeground="gray",
        )
        self.start_agentserial_button.pack(pady=1, side=tk.LEFT)

        # Stop Button
        self.stop_agent_button = tk.Button(
            frame_agent_button, 
            text="Stop micro-ROS Agent", 
            command=self.stop_ros2_agent, 
            state=tk.DISABLED,
            bg="red", 
            fg="white",
            activebackground="red2",
            activeforeground="white",
            disabledforeground="gray"
        )
        self.stop_agent_button.pack(pady=1, side=tk.LEFT)

    # --- NEW: Pump Logic ---
    def validate_pwm(self, var):
        try:
            val = int(var.get())
            if val < 0: val = 0
            if val > 100: val = 100
            return val
        except ValueError:
            return 0

    def toggle_supply_pump(self):
        if not self.ros2_node:
            messagebox.showwarning("Warning", "ROS2 Node is not initialized.")
            return

        if not self.is_supply_on:
            # Turn ON
            pwm = self.validate_pwm(self.supply_pwm_var)
            self.ros2_node.publish_supply_pump(pwm)
            self.btn_supply_toggle.config(text="ON", bg="green")
            self.is_supply_on = True
            self.terminal_output.insert(tk.END, f"\n[GUI] Supply Pump ON: {pwm}%", ("white",))
        else:
            # Turn OFF
            self.ros2_node.publish_supply_pump(0)
            self.btn_supply_toggle.config(text="OFF", bg="red")
            self.is_supply_on = False
            self.terminal_output.insert(tk.END, f"\n[GUI] Supply Pump OFF", ("white",))
        self.terminal_output.see(tk.END)

    def toggle_suction_pump(self):
        if not self.ros2_node:
            messagebox.showwarning("Warning", "ROS2 Node is not initialized.")
            return

        if not self.is_suction_on:
            # Turn ON
            pwm = self.validate_pwm(self.suction_pwm_var)
            self.ros2_node.publish_suction_pump(pwm)
            self.btn_suction_toggle.config(text="ON", bg="green")
            self.is_suction_on = True
            self.terminal_output.insert(tk.END, f"\n[GUI] Suction Pump ON: {pwm}%", ("white",))
        else:
            # Turn OFF
            self.ros2_node.publish_suction_pump(0)
            self.btn_suction_toggle.config(text="OFF", bg="red")
            self.is_suction_on = False
            self.terminal_output.insert(tk.END, f"\n[GUI] Suction Pump OFF", ("white",))
        self.terminal_output.see(tk.END)

    def update_pump_if_on(self, pump_type):
        if not self.ros2_node: return
        
        if pump_type == "supply" and self.is_supply_on:
            pwm = self.validate_pwm(self.supply_pwm_var)
            self.ros2_node.publish_supply_pump(pwm)
            self.terminal_output.insert(tk.END, f"\n[GUI] Updated Supply Pump: {pwm}%", ("white",))
            self.terminal_output.see(tk.END)
        
        elif pump_type == "suction" and self.is_suction_on:
            pwm = self.validate_pwm(self.suction_pwm_var)
            self.ros2_node.publish_suction_pump(pwm)
            self.terminal_output.insert(tk.END, f"\n[GUI] Updated Suction Pump: {pwm}%", ("white",))
            self.terminal_output.see(tk.END)
    
    # --- NEW: Monitoring Updates ---
    def update_raw_pos(self, value):
        # Update in main thread
        self.root.after(0, lambda: self.lbl_raw_pos.config(text=f"Raw Pos: {value}"))

    def update_rev_count(self, value):
        # Update UI
        self.root.after(0, lambda: self.lbl_rev_count.config(text=f"Rev: {value:.2f}"))
        
        # Check Limit Logic
        if not self.limit_triggered:
            # Check Upper Limit
            if self.use_upper_limit_var.get():
                try:
                    limit = float(self.upper_limit_var.get())
                    if value >= limit:
                        # Check if moving vertically (positive speed usually means increasing revs, depends on motor)
                        # Actually simpler: just stop if out of bounds and moving.
                        if self.motor_speed_slider.get() != 0:
                            self.stop_motor_due_to_limit(limit, "Max")
                except ValueError:
                    pass

            # Check Lower Limit
            if not self.limit_triggered and self.use_lower_limit_var.get():
                try:
                    limit = float(self.lower_limit_var.get())
                    if value <= limit:
                        if self.motor_speed_slider.get() != 0:
                            self.stop_motor_due_to_limit(limit, "Min")
                except ValueError:
                    pass

    def stop_motor_due_to_limit(self, limit, limit_type):
        if self.ros2_node:
            self.limit_triggered = True
            self.ros2_node.publish_motor_speed(0)
            self.root.after(0, lambda: self.handle_limit_stop_gui(limit, limit_type))

    def handle_limit_stop_gui(self, limit, limit_type):
        self.motor_speed_slider.set(0)
        self.terminal_output.insert(tk.END, f"\n[Limit] Motor stopped! {limit_type} limit {limit} reached.", ("red",))
        self.terminal_output.see(tk.END)
        messagebox.showwarning("Limit Reached", f"Motor stopped because {limit_type} revolution limit ({limit}) was reached.")
    
    def reset_limit_state(self):
        self.limit_triggered = False
        self.terminal_output.insert(tk.END, f"\n[Limit] State reset. Motor can move again.", ("green",))
        self.terminal_output.see(tk.END)
    # ------------------

    # --- NEW: Enable/Disable Entry based on Checkbox ---
    def toggle_timer_entry(self):
        if self.use_timer_var.get():
            self.ent_duration.config(state=tk.NORMAL)
        else:
            self.ent_duration.config(state=tk.DISABLED)

    # --- NEW: Thread Logic for Auto-Stop ---
    def motor_stop_thread(self, duration, current_timer_id):
        # Wait for the specified time
        time.sleep(duration)
        
        # Check if this timer is still the active one
        # (If user clicked "Set Speed" again, motor_timer_id would have changed)
        if self.motor_timer_id == current_timer_id:
            if self.ros2_node:
                self.ros2_node.publish_motor_speed(0)
                # Safely update GUI from thread
                self.root.after(0, self.update_terminal_after_stop)

    def update_terminal_after_stop(self):
        self.terminal_output.insert(tk.END, f"\n[Timer] Auto-stop triggered. Motor Speed: 0", ("yellow",))
        self.terminal_output.see(tk.END)
        self.motor_speed_slider.set(0) # Reset slider to 0

    def send_motor_command(self):
        if self.ros2_node:
            value = self.motor_speed_slider.get()
            
            # Increment ID to invalidate any previous running timers
            self.motor_timer_id += 1
            
            # Publish start command
            self.ros2_node.publish_motor_speed(value)
            
            log_msg = f"\n[GUI] Sent Motor Speed: {value}"
            
            # Check if timer is enabled
            if self.use_timer_var.get():
                try:
                    duration = float(self.timer_duration_var.get())
                    if duration > 0:
                        log_msg += f" (Auto-stop in {duration}s)"
                        # Start the thread
                        threading.Thread(
                            target=self.motor_stop_thread, 
                            args=(duration, self.motor_timer_id), 
                            daemon=True
                        ).start()
                    else:
                        messagebox.showwarning("Warning", "Duration must be > 0. Running continuously.")
                except ValueError:
                    messagebox.showerror("Error", "Invalid time duration. Please enter a number.")
                    return

            self.terminal_output.insert(tk.END, log_msg, ("white",))
            self.terminal_output.see(tk.END)
        else:
            messagebox.showwarning("Warning", "ROS2 Node is not initialized yet.")
    
    def start_ros2_agent(self):
        if self.process is None:
            self.process = subprocess.Popen(
                ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "udp4", "--port", "8888"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
            )
            self.start_agent_button.config(state=tk.DISABLED, bg="dodgerblue4")
            self.start_agentserial_button.config(state=tk.DISABLED, bg="dodgerblue4")
            self.stop_agent_button.config(state=tk.NORMAL)
            self.ins_label.config(text="Micro-ros agent has selected to Wi-Fi mode")
            threading.Thread(target=self.read_terminal_output, daemon=True).start()

    def start_ros2_agent_serial(self):
        if self.process is None:
            self.process = subprocess.Popen(
                ["ros2", "run", "micro_ros_agent", "micro_ros_agent", "serial", "--dev", "/dev/ttyUSB0"],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
            )
            self.start_agentserial_button.config(state=tk.DISABLED, bg="dodgerblue4")
            self.start_agent_button.config(state=tk.DISABLED, bg="dodgerblue4")
            self.stop_agent_button.config(state=tk.NORMAL)
            self.ins_label.config(text="Micro-ros agent has selected to Serial mode")
            threading.Thread(target=self.read_terminal_output, daemon=True).start()

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
                self.terminal_output.insert(tk.END, cleaned_line, (color,))
                self.terminal_output.tag_config(color, foreground=color)
                self.terminal_output.see(tk.END)

    def stop_ros2_agent(self):
        os.system("pkill -f micro_ros_agent")
        self.terminal_output.delete("1.0", tk.END)
        self.process = None
        self.start_agent_button.config(state=tk.NORMAL, bg="dodgerblue3")
        self.start_agentserial_button.config(state=tk.NORMAL, bg="dodgerblue3")
        self.stop_agent_button.config(state=tk.DISABLED)
        self.ins_label.config(text="Micro-ros agent is not running [please start agent within 5 second after power the sensor modules]")

    def start(self, ros2_node):
        self.ros2_node = ros2_node
        self.root.mainloop()
        
    def on_closing(self):
        self.stop_ros2_agent()
        self.root.destroy()

def main():
    timestamp_topics = [f"Visens/Current1/TimeStamp"] + [f"Visens/Vibration{i}/TimeStamp" for i in range(1, 11)]
    status_topics = [f"Visens/Current1/Status"] + [f"Visens/Vibration{i}/Status" for i in range(1, 11)]

    rclpy.init()
    node = TopicMonitorNode(timestamp_topics, status_topics, None, None)
    gui = MonitorGUI(timestamp_topics, status_topics)
    node.set_gui(gui) # Link GUI to Node

    def ros2_thread():
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()

    def trigger_callback(value):
        print(f"Logging {'Started' if value else 'Stopped'}")

    node.trigger_callback = trigger_callback

    threading.Thread(target=ros2_thread, daemon=True).start()
    gui.start(node)

if __name__ == "__main__":
    main()