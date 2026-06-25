import tkinter as tk
from tkinter import ttk, messagebox
from datetime import datetime


class TwoLiftControlGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Two Lift Control Panel")
        self.root.geometry("1550x820+0+0")
        self.root.configure(bg="#eaf0f5")

        self.lift1_pos = 100
        self.lift1_turn = 0
        self.lift1_tilt = 0
        self.lift2_turn = 0
        self.emergency_active = False

        self.create_header()
        self.create_layout()
        self.draw_visualization()

    def create_header(self):
        header = tk.Frame(self.root, bg="#061426", height=55)
        header.pack(fill="x")

        tk.Label(
            header,
            text="TWO LIFT CONTROL PANEL WITH VISUALIZATION",
            font=("Arial", 22, "bold"),
            bg="#061426",
            fg="white"
        ).pack(side="left", padx=30, pady=10)

        tk.Label(
            header,
            text="● CONNECTED",
            font=("Arial", 12, "bold"),
            bg="#061426",
            fg="#00cc44"
        ).pack(side="right", padx=25)

    def create_layout(self):
        main = tk.Frame(self.root, bg="#eaf0f5")
        main.pack(fill="both", expand=True, padx=15, pady=10)

        self.left = tk.LabelFrame(main, text="LIFT 1 CONTROL", fg="green",
                                  font=("Arial", 11, "bold"), bg="#f8fbff")
        self.left.pack(side="left", fill="y", padx=8)

        self.center = tk.Frame(main, bg="#eaf0f5")
        self.center.pack(side="left", fill="both", expand=True, padx=8)

        self.right = tk.LabelFrame(main, text="LIFT 2 CONTROL", fg="blue",
                                   font=("Arial", 11, "bold"), bg="#f8fbff")
        self.right.pack(side="right", fill="y", padx=8)

        self.create_lift1_controls()
        self.create_center()
        self.create_lift2_controls()

    def make_button(self, parent, text, color, command, width=18):
        return tk.Button(
            parent, text=text, bg=color, fg="white",
            font=("Arial", 10, "bold"),
            width=width, height=2,
            command=command
        )

    def create_lift1_controls(self):
        tk.Label(self.left, text="Lift 1: 3 Motor System",
                 font=("Arial", 12, "bold"), bg="#f8fbff").pack(pady=10)

        self.section(self.left, "ORIENTAL MOTOR 1 : LIFTING", [
            ("⬆ LIFT UP", "green", self.lift1_up),
            ("⬇ LIFT DOWN", "green", self.lift1_down)
        ])

        self.section(self.left, "ORIENTAL MOTOR 2 : TURNING", [
            ("↻ TURN CW", "#0057b8", self.lift1_turn_cw),
            ("↺ TURN CCW", "#0057b8", self.lift1_turn_ccw)
        ])

        self.section(self.left, "MAXON MOTOR : TILTING", [
            ("⬅ TILT LEFT", "#7a2fc2", self.lift1_tilt_left),
            ("➡ TILT RIGHT", "#7a2fc2", self.lift1_tilt_right)
        ])

        self.make_button(self.left, "■ STOP LIFT 1", "#e60000",
                         lambda: self.stop_lift("Lift 1")).pack(pady=15)

        specs = """
LIFT 1 SPECIFICATIONS

Oriental Motor 1
Function     : Lift Up / Down
Voltage      : 24 V AC
Power        : 60 W
Gear Ratio   : 10:1

Oriental Motor 2
Function     : Turn CW / CCW
Voltage      : 24 V AC
Speed        : 3000 RPM

Maxon Motor
Function     : Tilt Left / Right
Voltage      : 24 V DC
Power        : 50 W
Encoder      : Incremental Encoder
"""
        tk.Label(self.left, text=specs, justify="left",
                 font=("Arial", 12), bg="#f8fbff").pack(padx=15)

    def create_lift2_controls(self):
        tk.Label(self.right, text="Lift 2: Manual Lift + Turning Motor",
                 font=("Arial", 12, "bold"), bg="#f8fbff").pack(pady=10)

        self.section(self.right, "MANUAL LIFT", [
            ("⬆ MANUAL LIFT UP", "green", lambda: self.log("Lift 2 manual lift up")),
            ("⬇ MANUAL LIFT DOWN", "green", lambda: self.log("Lift 2 manual lift down"))
        ], width=16)

        self.section(self.right, "ORIENTAL MOTOR 3 : TURNING", [
            ("↻ TURN CW", "#0057b8", self.lift2_turn_cw),
            ("↺ TURN CCW", "#0057b8", self.lift2_turn_ccw)
        ], width=16)

        self.make_button(self.right, "■ STOP LIFT 2", "#e60000",
                         lambda: self.stop_lift("Lift 2")).pack(pady=20)

        specs = """
LIFT 2 SPECIFICATIONS

Manual Lift
Function     : Manual Up / Down
Control      : Operator Controlled

Oriental Motor 3
Function     : Turn CW / CCW
Voltage      : 24 V AC
Power        : 60 W
Speed        : 3000 RPM
Gear Ratio   : 10:1
"""
        tk.Label(self.right, text=specs, justify="left",
                 font=("Arial", 12), bg="#f8fbff").pack(padx=15)

        emergency = tk.LabelFrame(self.right, text="EMERGENCY STOP",
                                  fg="red", font=("Arial", 11, "bold"),
                                  bg="#f8fbff")
        emergency.pack(fill="x", padx=15, pady=20)

        tk.Label(emergency, text="⚠", font=("Arial", 36, "bold"),
                 fg="red", bg="#f8fbff").pack()

        self.make_button(emergency, "EMERGENCY STOP\nALL MOTORS",
                         "#e60000", self.emergency_stop, width=24).pack(pady=10)

    def section(self, parent, title, items, width=18):
        frame = tk.LabelFrame(parent, text=title, fg="#003399",
                              font=("Arial", 9, "bold"), bg="#f8fbff")
        frame.pack(fill="x", padx=12, pady=10)

        btn_frame = tk.Frame(frame, bg="#f8fbff")
        btn_frame.pack(pady=10)

        for text, color, command in items:
            self.make_button(btn_frame, text, color, command, width).pack(side="left", padx=5)

    def create_center(self):
        vis = tk.LabelFrame(self.center, text="LIFT VISUALIZATION",
                            fg="navy", font=("Arial", 12, "bold"), bg="#f8fbff")
        vis.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(vis, bg="white", height=410)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=10)

        monitor = tk.LabelFrame(self.center, text="REAL-TIME MONITORING",
                                fg="navy", font=("Arial", 12, "bold"), bg="#f8fbff")
        monitor.pack(fill="x", pady=8)

        columns = ("Motor", "Function", "Speed", "Current", "Position", "Status")
        self.table = ttk.Treeview(monitor, columns=columns, show="headings", height=4)

        for col in columns:
            self.table.heading(col, text=col)
            self.table.column(col, width=120, anchor="center")

        self.table.pack(fill="x", padx=10, pady=5)

        log_frame = tk.LabelFrame(self.center, text="SYSTEM STATUS / LOG",
                                  fg="navy", font=("Arial", 12, "bold"), bg="#f8fbff")
        log_frame.pack(fill="x")

        self.status_label = tk.Label(log_frame, text="Status: READY",
                                     fg="green", bg="#f8fbff",
                                     font=("Arial", 11, "bold"))
        self.status_label.pack(anchor="w", padx=15, pady=5)

        self.log_box = tk.Text(log_frame, height=6, font=("Courier", 10))
        self.log_box.pack(fill="x", padx=10, pady=5)

    
    def draw_visualization(self):
        self.canvas.delete("all")

        # Limit values for safe visualization
        self.lift1_pos = max(20, min(self.lift1_pos, 180))
        self.lift1_turn = self.lift1_turn % 360
        self.lift2_turn = self.lift2_turn % 360

        # Convert lift position to canvas Y position
        lift1_top_y = 270 - self.lift1_pos
        lift1_base_y = 270

        lift2_top_y = 160
        lift2_base_y = 270

        # =========================
        # LIFT 1 VISUALIZATION
        # =========================
        self.canvas.create_text(
            220, 35,
            text="LIFT 1",
            font=("Arial", 18, "bold"),
            fill="green"
        )

        # Lift 1 base
        self.canvas.create_rectangle(
            170, 270, 270, 340,
            fill="#222222",
            outline="black"
        )

        # Lift 1 vertical lifting column
        self.canvas.create_rectangle(
            195, lift1_top_y, 245, lift1_base_y,
            fill="#bfc7d1",
            outline="black"
        )

        # Turn motor circular head
        self.canvas.create_oval(
            165, lift1_top_y - 55,
            275, lift1_top_y + 55,
            outline="green",
            width=4
        )

        # Red center shaft
        self.canvas.create_line(
            220, lift1_top_y - 80,
            220, lift1_top_y,
            fill="red",
            width=6
        )

        # Turning direction line
        self.canvas.create_arc(
            175, lift1_top_y - 45,
            265, lift1_top_y + 45,
            start=self.lift1_turn,
            extent=80,
            style="arc",
            outline="blue",
            width=3
        )

        # Tilt indicator
        tilt_x = 220 + self.lift1_tilt
        self.canvas.create_line(
            220, lift1_top_y - 20,
            tilt_x, lift1_top_y - 65,
            fill="purple",
            width=4,
            arrow="last"
        )

        # Lift 1 labels
        self.canvas.create_text(
            80, 115,
            text="LIFT UP",
            fill="green",
            font=("Arial", 11, "bold")
        )

        self.canvas.create_text(
            80, 215,
            text="LIFT DOWN",
            fill="red",
            font=("Arial", 11, "bold")
        )

        self.canvas.create_text(
            340, 135,
            text=f"TURN ANGLE: {self.lift1_turn}°",
            fill="blue",
            font=("Arial", 10, "bold")
        )

        self.canvas.create_text(
            340, 165,
            text=f"TILT ANGLE: {self.lift1_tilt}°",
            fill="purple",
            font=("Arial", 10, "bold")
        )

        # Lift 1 parameter box
        self.canvas.create_rectangle(
            120, 355, 330, 400,
            outline="#888"
        )

        self.canvas.create_text(
            225, 372,
            text=f"Lift Position: {self.lift1_pos} mm",
            fill="green",
            font=("Arial", 10, "bold")
        )

        self.canvas.create_text(
            225, 390,
            text=f"Turn: {self.lift1_turn}°   Tilt: {self.lift1_tilt}°",
            fill="blue",
            font=("Arial", 10, "bold")
        )

        # Divider
        self.canvas.create_line(
            430, 50, 430, 400,
            fill="#b0b0b0",
            width=2
        )

        # =========================
        # LIFT 2 VISUALIZATION
        # =========================
        self.canvas.create_text(
            650, 35,
            text="LIFT 2",
            font=("Arial", 18, "bold"),
            fill="blue"
        )

        # Lift 2 base
        self.canvas.create_rectangle(
            600, 270, 700, 340,
            fill="#222222",
            outline="black"
        )

        # Lift 2 manual column
        self.canvas.create_rectangle(
            620, lift2_top_y, 680, lift2_base_y,
            fill="#aab2bd",
            outline="black"
        )

        # Lift 2 turning head
        self.canvas.create_oval(
            590, lift2_top_y - 30,
            710, lift2_top_y + 20,
            outline="blue",
            width=4
        )

        # Lift 2 turning direction arc
        self.canvas.create_arc(
            600, lift2_top_y - 25,
            700, lift2_top_y + 15,
            start=self.lift2_turn,
            extent=80,
            style="arc",
            outline="green",
            width=3
        )

        # Lift 2 center shaft
        self.canvas.create_line(
            650, lift2_top_y - 55,
            650, lift2_top_y + 10,
            fill="red",
            width=6
        )

        # Lift 2 labels
        self.canvas.create_text(
            535, 120,
            text="MANUAL UP",
            fill="green",
            font=("Arial", 11, "bold")
        )

        self.canvas.create_text(
            535, 220,
            text="MANUAL DOWN",
            fill="red",
            font=("Arial", 11, "bold")
        )

        self.canvas.create_text(
            770, 140,
            text=f"TURN ANGLE: {self.lift2_turn}°",
            fill="blue",
            font=("Arial", 10, "bold")
        )

        # Lift 2 parameter box
        self.canvas.create_rectangle(
            545, 355, 760, 400,
            outline="#888"
        )

        self.canvas.create_text(
            652, 372,
            text="Manual Lift: Operator Controlled",
            fill="green",
            font=("Arial", 10, "bold")
        )

        self.canvas.create_text(
            652, 390,
            text=f"Turn Angle: {self.lift2_turn}°",
            fill="blue",
            font=("Arial", 10, "bold")
        )

        self.update_table()

    def update_table(self):
        for item in self.table.get_children():
            self.table.delete(item)

        self.table.insert("", "end", values=("Oriental 1", "Lift", "1250", "0.45 A",
                                             f"{self.lift1_pos} mm", "RUN/IDLE"))
        self.table.insert("", "end", values=("Oriental 2", "Turn", "1500", "0.52 A",
                                             f"{self.lift1_turn}°", "RUN/IDLE"))
        self.table.insert("", "end", values=("Maxon", "Tilt", "2150", "0.68 A",
                                             f"{self.lift1_tilt}°", "RUN/IDLE"))
        self.table.insert("", "end", values=("Oriental 3", "Turn", "1400", "0.50 A",
                                             f"{self.lift2_turn}°", "RUN/IDLE"))

    def lift1_up(self):
        self.lift1_pos += 10
        if self.lift1_pos > 180:
            self.lift1_pos = 180
        self.status_label.config(text="Status: Lift 1 Moving Up", fg="green")
        self.log("Lift 1: Lift UP")
        self.draw_visualization()

    def lift1_down(self):
        self.lift1_pos -= 10
        if self.lift1_pos < 20:
            self.lift1_pos = 20
        self.status_label.config(text="Status: Lift 1 Moving Down", fg="green")
        self.log("Lift 1: Lift DOWN")
        self.draw_visualization()

    def lift1_turn_cw(self):
        self.lift1_turn -= 5
        self.status_label.config(
            text=f"Status: Lift 1 Turning CW ({self.lift1_turn}°)",
            fg="blue"
        )
        self.log(f"Lift 1: Turn CW -> {self.lift1_turn}°")
        self.draw_visualization()


    def lift1_turn_ccw(self):
        self.lift1_turn += 5
        self.status_label.config(
            text=f"Status: Lift 1 Turning CCW ({self.lift1_turn}°)",
            fg="blue"
        )
        self.log(f"Lift 1: Turn CCW -> {self.lift1_turn}°")
        self.draw_visualization()

    def lift1_tilt_left(self):
        self.lift1_tilt -= 5
        if self.lift1_tilt < -45:
            self.lift1_tilt = -45
        self.status_label.config(text="Status: Maxon Tilt Left", fg="purple")
        self.log("Lift 1: Tilt LEFT")
        self.draw_visualization()

    def lift1_tilt_right(self):
        self.lift1_tilt += 5
        if self.lift1_tilt > 45:
            self.lift1_tilt = 45
        self.status_label.config(text="Status: Maxon Tilt Right", fg="purple")
        self.log("Lift 1: Tilt RIGHT")
        self.draw_visualization()

    def lift2_turn_cw(self):
        self.lift2_turn -= 5
        self.status_label.config(
            text=f"Status: Lift 2 Turning CW ({self.lift2_turn}°)",
            fg="blue"
        )
        self.log(f"Lift 2: Turn CW -> {self.lift2_turn}°")
        self.draw_visualization()

    def lift2_turn_ccw(self):
        self.lift2_turn += 5
        self.status_label.config(
            text=f"Status: Lift 2 Turning CCW ({self.lift2_turn}°)",
            fg="blue"
        )
        self.log(f"Lift 2: Turn CCW -> {self.lift2_turn}°")
        self.draw_visualization()

    def stop_lift(self, lift_name):
        self.status_label.config(text=f"Status: {lift_name} STOPPED", fg="red")
        self.log(f"{lift_name}: stopped")

    def emergency_stop(self):
        self.emergency_active = True
        self.status_label.config(text="Status: EMERGENCY STOP ACTIVE", fg="red")
        self.log("EMERGENCY STOP: All motors stopped")
        messagebox.showwarning("Emergency Stop", "All Lift Motors Stopped!")

    def log(self, message):
        now = datetime.now().strftime("%H:%M:%S")
        self.log_box.insert("end", f"[{now}] {message}\n")
        self.log_box.see("end")


if __name__ == "__main__":
    root = tk.Tk()
    app = TwoLiftControlGUI(root)
    root.mainloop()