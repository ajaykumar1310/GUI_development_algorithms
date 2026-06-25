#!/usr/bin/env python3

import tkinter as tk
from tkinter import messagebox
from datetime import datetime
import rclpy
import os
import signal
import subprocess
from geometry_msgs.msg import Twist
from PIL import Image, ImageTk


class AMRSLAMGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("AMR SLAM Control Panel")
        self.root.geometry("1450x850+70+40")
        self.root.configure(bg="#f2f2f2")

        self.front_lidar_process = None
        self.rear_lidar_process = None

        self.front_crop_process = None
        self.rear_crop_process = None
        self.merge_process = None

        self.roboteq_process = None
        self.diff_drive_process = None

        self.odom_process = None

        self.slam_process = None

        self.front_tf_process = None
        self.rear_tf_process = None

        # self.front_tf_echo_process = None
        # self.rear_tf_echo_process = None

        rclpy.init()
        self.ros_node = rclpy.create_node("amr_slam_gui_teleop")
        self.cmd_pub = self.ros_node.create_publisher(Twist, "/cmd_vel", 10)

        # self.create_header()
        
        # header = tk.Frame(root, bg="#1f2933")
        # header.pack(fill="x")
        self.header = self.create_header()

        main = tk.Frame(root, bg="#f2f2f2")
        main.pack(fill="both", expand=True, padx=10, pady=5)

        self.left_panel = tk.Frame(main, bg="#f2f2f2", width=330)
        self.left_panel.pack(side="left", fill="y", padx=5)

        self.center_panel = tk.Frame(main, bg="#f2f2f2", width=700)
        self.center_panel.pack(side="left", fill="both", expand=True, padx=5)

        self.right_panel = tk.Frame(main, bg="#f2f2f2", width=330)
        self.right_panel.pack(side="right", fill="y", padx=5)

        self.front_lidar_process = None
        self.rear_lidar_process = None

        self.time_label = tk.Label(
            self.header,
            font=("Arial", 12, "bold"),
            bg="#1f2933",
            fg="white"
        )

        self.time_label.pack(side="right", padx=30)

        self.update_time()

        self.make_control_panel()
        self.make_teleop_panel()
        self.make_emergency_panel()

        self.make_rviz_view()
        self.make_laser_view()

        self.make_system_status()
        self.make_topic_status()
        self.make_odometry_info()
    
        self.make_system_log()


    def update_time(self):

        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        self.time_label.config(
            text=f"System Time: {now}"
        )

        self.root.after(1000, self.update_time)


    def create_header(self):
        header = tk.Frame(self.root, bg="#1f2933", height=70)
        header.pack(fill="x")

        title_frame = tk.Frame(header, bg="#1f2933")
        title_frame.pack(side="left", expand=True)

        # Load logo
        logo = Image.open(
            "/home/anytoy/slam_ws/src/amr_control/amr_control/images/anytoy_logo_eng.png"
        )

        # Resize logo
        logo = logo.resize((220, 50))

        # Convert for Tkinter
        self.logo_img = ImageTk.PhotoImage(logo)

        # Display logo
        tk.Label(
            title_frame,
            image=self.logo_img,
            bg="#1f2933"
        ).pack(side="left", padx=15)

        tk.Label(
            header,
            text="AMR SLAM Control Panel",
            font=("Arial", 26, "bold"),
            bg="#1f2933",
            fg="white"
        ).pack(side="left", padx=250)


        tk.Label(
            header,
            text="● ACTIVE",
            font=("Arial", 13, "bold"),
            bg="#1f2933",
            fg="#39b54a"
        ).pack(side="right", padx=20)
    
        return header

    def make_section(self, parent, title):
        frame = tk.LabelFrame(
            parent,
            text=title,
            font=("Arial", 11, "bold"),
            fg="#1f4ed8",
            bg="#f2f2f2",
            labelanchor="n",
            padx=10,
            pady=10,
            bd=2,
            relief="groove"
        )
        frame.pack(fill="both", padx=3, pady=5)
        return frame

    def button(self, parent, text, bg, fg, command):
        tk.Button(
            parent,
            text=text,
            font=("Arial", 12, "bold"),
            bg=bg,
            fg=fg,
            width=28,
            height=2,
            command=command
        ).pack(pady=4)

    def make_control_panel(self):
        frame = self.make_section(self.left_panel, "CONTROL PANEL")

        # self.button(frame, "▶   START ALL SYSTEMS", "#238823", "white", self.start_all)
        self.button(frame, "📡   START FRONT LIDAR", "#dcecff", "black", self.start_front_lidar)
        self.button(frame, "📡   START REAR LIDAR", "#dcecff", "black", self.start_rear_lidar)
        self.button(frame, "✂   START CROP NODES", "#dcecff", "black", self.start_crop_nodes)
        self.button(frame, "🔀   START MERGE NODE", "#dcecff", "black", self.start_merge_node)
        self.button(frame, "⚙   START ODOMETRY", "#dcecff", "black", self.start_odometry)
        self.button(frame, "⚙   START TF FRAMES", "#dcecff", "black", self.start_tf)
        self.button(frame, "🗺   START SLAM TOOLBOX", "#dcecff", "black", self.start_slam_toolbox)
        self.button(frame, "🟪   OPEN RVIZ", "#dcecff", "black", self.open_rviz)
        self.button(frame, "💾   SAVE MAP", "#dcecff", "black", self.save_map)
        self.button(frame, "■   STOP ALL SYSTEMS", "#d83228", "white", self.stop_all)

    def make_teleop_panel(self):
        frame = self.make_section(self.left_panel, "TELEOP CONTROL")

        teleop = tk.Frame(frame, bg="#f2f2f2")
        teleop.pack()

        buttons = tk.Frame(teleop, bg="#f2f2f2")
        buttons.grid(row=0, column=0, padx=10)

        tk.Button(buttons, text="⬆", font=("Arial", 18, "bold"), width=3, command=self.forward).grid(row=0, column=1)
        tk.Button(buttons, text="⬅", font=("Arial", 18, "bold"), width=3, command=self.left).grid(row=1, column=0)
        tk.Button(buttons, text="■", font=("Arial", 18, "bold"), fg="red", width=3, command=self.robot_stop).grid(row=1, column=1)
        tk.Button(buttons, text="➡", font=("Arial", 18, "bold"), width=3, command=self.right).grid(row=1, column=2)
        tk.Button(buttons, text="⬇", font=("Arial", 18, "bold"), width=3, command=self.backward).grid(row=2, column=1)

        speeds = tk.Frame(teleop, bg="#f2f2f2")
        speeds.grid(row=0, column=1, padx=20)

        tk.Label(speeds, text="Linear (m/s)", bg="#f2f2f2").pack()
        self.linear = tk.Entry(speeds, width=8, justify="center", font=("Arial", 12))
        self.linear.insert(0, "0.20")
        self.linear.pack(pady=5)

        tk.Label(speeds, text="Angular (rad/s)", bg="#f2f2f2").pack()
        self.angular = tk.Entry(speeds, width=8, justify="center", font=("Arial", 12))
        self.angular.insert(0, "0.30")
        self.angular.pack(pady=5)

    def make_emergency_panel(self):
        frame = self.make_section(self.left_panel, "EMERGENCY")

        tk.Button(
            frame,
            text="⚠   EMERGENCY STOP",
            font=("Arial", 15, "bold"),
            bg="#ffd43b",
            fg="red",
            width=25,
            height=1,
            command=self.emergency_stop
        ).pack(pady=10)

    def make_rviz_view(self):
        frame = self.make_section(self.center_panel, "RVIZ LIVE VIEW (Mapping)")

        self.rviz_canvas = tk.Canvas(frame, bg="#263238", height=410)
        self.rviz_canvas.pack(fill="both", expand=True)

        self.rviz_canvas.create_text(
            350, 120,
            text="RViz is running in separate window",
            fill="white",
            font=("Arial", 20, "bold")
        )

        self.rviz_canvas.create_text(
            350, 180,
            text="Use OPEN RVIZ button to visualize:",
            fill="#90caf9",
            font=("Arial", 14, "bold")
        )

        self.rviz_canvas.create_text(
            350, 220,
            text="/map   /tf   /odom   /merge/scan",
            fill="white",
            font=("Arial", 13)
        )

        self.rviz_canvas.create_text(
            350, 280,
            text="GUI is used for control. RViz is used for visualization.",
            fill="#cfd8dc",
            font=("Arial", 12)
        )

    def make_laser_view(self):
        frame = self.make_section(self.center_panel, "LIVE LASER SCAN (Merged)")

        canvas = tk.Canvas(frame, bg="#263238", height=250)
        canvas.pack(fill="both", expand=True)

        canvas.create_text(350, 30, text="Dummy LaserScan View", fill="#90caf9", font=("Arial", 16, "bold"))

        for i in range(80, 620, 20):
            canvas.create_oval(i, 130, i + 3, 133, fill="red", outline="red")

        canvas.create_text(350, 220, text="/merge/scan data will appear here later", fill="white")

    def make_system_status(self):
        frame = self.make_section(self.right_panel, "SYSTEM STATUS")

        items = [
            "Front LiDAR", "Rear LiDAR", "Front Crop Node",
            "Rear Crop Node", "Merge Scan Node", "Odometry",
            "SLAM Toolbox", "TF (base_link)"
        ]

        for item in items:
            row = tk.Frame(frame, bg="#f2f2f2")
            row.pack(fill="x", pady=2)

            tk.Label(row, text=f"●  {item}", fg="#39b54a", bg="#f2f2f2",
                     font=("Arial", 10, "bold")).pack(side="left")

            tk.Label(row, text="Running", fg="green", bg="#f2f2f2",
                     font=("Arial", 10)).pack(side="right")

    def make_topic_status(self):
        frame = self.make_section(self.right_panel, "TOPIC STATUS")

        topics = [
            ("/front/scan", "10.0 Hz"),
            ("/rear/scan", "10.0 Hz"),
            ("/front/scan_crop", "10.0 Hz"),
            ("/rear/scan_crop", "10.0 Hz"),
            ("/merge/scan", "10.0 Hz"),
            ("/odom", "20.0 Hz"),
            ("/map", "1.0 Hz"),
        ]

        for topic, hz in topics:
            row = tk.Frame(frame, bg="#f2f2f2")
            row.pack(fill="x", pady=2)

            tk.Label(row, text=topic, bg="#f2f2f2",
                     font=("Arial", 10)).pack(side="left")

            tk.Label(row, text=hz, fg="green", bg="#f2f2f2",
                     font=("Arial", 10)).pack(side="right")

    def make_odometry_info(self):
        frame = self.make_section(self.right_panel, "ODOMETRY INFO")

        data = [
            ("Position X (m):", "1.234"),
            ("Position Y (m):", "2.567"),
            ("Yaw (rad):", "1.570"),
            ("Linear Vel (m/s):", "0.05"),
            ("Angular Vel (rad/s):", "0.02"),
        ]

        for name, value in data:
            row = tk.Frame(frame, bg="#f2f2f2")
            row.pack(fill="x", pady=2)

            tk.Label(row, text=name, bg="#f2f2f2",
                     font=("Arial", 10)).pack(side="left")

            tk.Label(row, text=value, bg="#f2f2f2",
                     font=("Arial", 10)).pack(side="right")

    def make_system_log(self):
        frame = self.make_section(self.right_panel, "SYSTEM LOG")

        self.log_box = tk.Text(
            frame,
            height=10,
            bg="black",
            fg="#00ff00",
            font=("Consolas", 10)
        )
        self.log_box.pack(fill="both")

        self.log("[10:34:12] Front LiDAR Started")
        self.log("[10:34:13] Rear LiDAR Started")
        self.log("[10:34:14] Crop Nodes Started")
        self.log("[10:34:15] Merge Node Started")
        self.log("[10:34:16] Odometry Started")
        self.log("[10:34:17] SLAM Toolbox Started")
        self.log("[10:34:18] All Systems Running")

    def log(self, msg):
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)
        print("Dummy:", msg)

    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_pub.publish(msg)

    # def start_all(self): self.log("START ALL SYSTEMS")

    # stop process

    def stop_process(self, process, name):

        if process is not None:
            try:
                os.killpg(
                    os.getpgid(process.pid),
                    signal.SIGINT
                )

                self.log(f"{name} STOPPED")

            except Exception as e:
                self.log(f"ERROR STOPPING {name}: {e}")

        return None

    # def stop_all(self): self.log("STOP ALL SYSTEMS")

    def stop_all(self):

        self.robot_stop()

        # SLAM
        self.slam_process = self.stop_process(
            self.slam_process,
            "SLAM TOOLBOX"
        )

        # MERGE NODE
        self.merge_process = self.stop_process(
            self.merge_process,
            "MERGE NODE"
        )

        # CROP NODES
        self.front_crop_process = self.stop_process(
            self.front_crop_process,
            "FRONT CROP NODE"
        )

        self.rear_crop_process = self.stop_process(
            self.rear_crop_process,
            "REAR CROP NODE"
        )

        # BASE CONTROL
        self.odom_process = self.stop_process(
            self.odom_process,
            "ODOMETRY NODE"
        )

        self.diff_drive_process = self.stop_process(
            self.diff_drive_process,
            "DIFFERENTIAL DRIVE NODE"
        )

        self.roboteq_process = self.stop_process(
            self.roboteq_process,
            "ROBOTEQ DRIVER"
        )

        # LIDARS
        self.front_lidar_process = self.stop_process(
            self.front_lidar_process,
            "FRONT LIDAR"
        )

        self.rear_lidar_process = self.stop_process(
            self.rear_lidar_process,
            "REAR LIDAR"
        )

        # TF PUBLISHERS
        self.front_tf_process = self.stop_process(
            self.front_tf_process,
            "FRONT TF"
        )

        self.rear_tf_process = self.stop_process(
            self.rear_tf_process,
            "REAR TF"
        )

        # # TF ECHO
        # self.front_tf_echo_process = self.stop_process(
        #     self.front_tf_echo_process,
        #     "FRONT TF ECHO"
        # )

        # self.rear_tf_echo_process = self.stop_process(
        #     self.rear_tf_echo_process,
        #     "REAR TF ECHO"
        # )

        self.log("STOP ALL SYSTEMS")

       

    # def start_front_lidar(self): self.log("START FRONT LIDAR")
    # def start_rear_lidar(self): self.log("START REAR LIDAR")

    def start_front_lidar(self):
        if self.front_lidar_process is None:
            self.log("START FRONT LIDAR")

            self.front_lidar_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 launch sick_scan_xd sick_tim_5xx.launch.py "
                "hostname:=169.254.18.208 "
                "publish_tf:=false "
                "tf_publish_rate:=0.0 "
                "frame_id:=laser_front "
                "laserscan_topic:=/front/scan "
                "__ns:=/front",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("FRONT LIDAR STARTED")
        else:
            self.log("FRONT LIDAR ALREADY RUNNING")

    def start_rear_lidar(self):
        if self.rear_lidar_process is None:
            self.log("START REAR LIDAR")

            self.rear_lidar_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 launch sick_scan_xd sick_tim_5xx.launch.py "
                "hostname:=169.254.79.80 "
                "publish_tf:=false "
                "tf_publish_rate:=0.0 "
                "frame_id:=laser_rear "
                "laserscan_topic:=/rear/scan "
                "__ns:=/rear",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("REAR LIDAR STARTED")

        else:
            self.log("REAR LIDAR ALREADY RUNNING")

    # def start_crop_nodes(self): self.log("START CROP NODES")

    def start_crop_nodes(self):
        self.log("START CROP NODES")

        if self.front_crop_process is None:
            self.front_crop_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 run amr_control scan_crop_node --ros-args "
                "-r __node:=front_scan_crop_node "
                "-p input_topic:=/front/scan "
                "-p output_topic:=/front/scan_crop "
                "-p lower_angle:=-1.57 "
                "-p upper_angle:=1.57 "
                "-p min_range_cutoff:=0.30",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )
            self.log("FRONT CROP NODE STARTED")

        if self.rear_crop_process is None:
            self.rear_crop_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 run amr_control scan_crop_node --ros-args "
                "-r __node:=rear_scan_crop_node "
                "-p input_topic:=/rear/scan "
                "-p output_topic:=/rear/scan_crop "
                "-p lower_angle:=-1.57 "
                "-p upper_angle:=1.57 "
                "-p min_range_cutoff:=0.30",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )
            self.log("REAR CROP NODE STARTED")

    # def start_merge_node(self): self.log("START MERGE NODE")

    def start_merge_node(self):
        if self.merge_process is None:
            self.log("START MERGE NODE")

            self.merge_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 run amr_control dual_lidar_merger_node",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("MERGE NODE STARTED")
        else:
            self.log("MERGE NODE ALREADY RUNNING")

    # def start_odometry(self): self.log("START ODOMETRY")

    def start_odometry(self):

        self.log("START BASE CONTROL")

        # ROBOTEQ DRIVER
        if self.roboteq_process is None:

            self.roboteq_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 run amr_control roboteq_driver_node",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("ROBOTEQ DRIVER STARTED")

        # DIFFERENTIAL DRIVE
        if self.diff_drive_process is None:

            self.diff_drive_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 run amr_control differential_drive_node",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("DIFFERENTIAL DRIVE STARTED")

        # ODOMETRY
        if self.odom_process is None:

            self.odom_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "source ~/slam_ws/install/setup.bash && "
                "ros2 run amr_control odometry_node",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("ODOMETRY STARTED")


    # start TF (Transfer graph)

    def start_tf(self):

        self.log("START TF FRAMES")

        # FRONT TF
        if self.front_tf_process is None:

            self.front_tf_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "ros2 run tf2_ros static_transform_publisher "
                "0.40 -0.30 0 "
                "0 0 -0.3827 0.9239 "
                "base_link laser_front",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("FRONT TF STARTED")

        # # FRONT TF ECHO
        # if self.front_tf_echo_process is None:

        #     self.front_tf_echo_process = subprocess.Popen(
        #         "source /opt/ros/jazzy/setup.bash && "
        #         "ros2 run tf2_ros tf2_echo base_link laser_front",
        #         shell=True,
        #         executable="/bin/bash",
        #         preexec_fn=os.setsid
        #     )

        #     self.log("FRONT TF ECHO STARTED")

        # REAR TF
        if self.rear_tf_process is None:

            self.rear_tf_process = subprocess.Popen(
                "source /opt/ros/jazzy/setup.bash && "
                "ros2 run tf2_ros static_transform_publisher "
                "-0.40 0.30 0 "
                "0 0 0.9239 0.3827 "
                "base_link laser_rear",
                shell=True,
                executable="/bin/bash",
                preexec_fn=os.setsid
            )

            self.log("REAR TF STARTED")

        # # REAR TF ECHO
        # if self.rear_tf_echo_process is None:

        #     self.rear_tf_echo_process = subprocess.Popen(
        #         "source /opt/ros/jazzy/setup.bash && "
        #         "ros2 run tf2_ros tf2_echo base_link laser_rear",
        #         shell=True,
        #         executable="/bin/bash",
        #         preexec_fn=os.setsid
        #     )

        #     self.log("REAR TF ECHO STARTED")


    # def start_slam_toolbox(self): self.log("START SLAM TOOLBOX")

    def start_slam_toolbox(self):

        if self.slam_process is not None or self.ros_node_exists("/slam_toolbox"):
            self.log("SLAM TOOLBOX ALREADY RUNNING")
            return

        self.log("START SLAM TOOLBOX")

        self.slam_process = subprocess.Popen(
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/slam_ws/install/setup.bash && "
            "ros2 launch slam_toolbox online_async_launch.py "
            "slam_params_file:=/home/anytoy/slam_ws/slam_toolbox_params.yaml",
            shell=True,
            executable="/bin/bash",
            preexec_fn=os.setsid
        )

        self.log("SLAM TOOLBOX STARTED")

    def stop_process(self, process, name):
        if process is not None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGINT)
                self.log(f"{name} STOPPED")
            except Exception as e:
                self.log(f"ERROR STOPPING {name}: {e}")
        return None

    # def open_rviz(self): self.log("OPEN RVIZ")

    def open_rviz(self):
        self.log("OPEN RVIZ")

        subprocess.Popen(
            "source /opt/ros/jazzy/setup.bash && "
            "source ~/slam_ws/install/setup.bash && "
            "rviz2",
            shell=True,
            executable="/bin/bash"
        )

    def save_map(self):
        self.log("SAVE MAP")
        messagebox.showinfo("SAVE MAP", "Dummy map saved successfully.")

    def forward(self):
        v = float(self.linear.get())
        self.send_cmd_vel(v, 0.0)
        self.log(f"MOVE FORWARD  linear={v}")

    def backward(self):
        v = float(self.linear.get())
        self.send_cmd_vel(-v, 0.0)
        self.log(f"MOVE BACKWARD  linear={-v}")

    def left(self):
        w = float(self.angular.get())
        self.send_cmd_vel(0.0, w)
        self.log(f"TURN LEFT  angular={w}")

    def right(self):
        w = float(self.angular.get())
        self.send_cmd_vel(0.0, -w)
        self.log(f"TURN RIGHT  angular={-w}")

    def robot_stop(self):
        self.send_cmd_vel(0.0, 0.0)
        self.log("ROBOT STOP")

    def emergency_stop(self):

        # Stop robot motion
        self.robot_stop()

        self.log("EMERGENCY STOP ACTIVATED")

        # Kill all ROS nodes immediately
        subprocess.run(
            "pkill -9 -f differential_drive_node",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f roboteq_driver_node",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f odometry_node",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f dual_lidar_merger_node",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f scan_crop_node",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f sick_scan_xd",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f sick_tim_5xx",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f slam_toolbox",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f sick",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f static_transform_publisher",
            shell=True
        )

        subprocess.run(
            "pkill -9 -f tf2_echo",
            shell=True
        )

        self.log("ALL NODES FORCE STOPPED")

        messagebox.showerror(
            "EMERGENCY STOP",
            "EMERGENCY STOP ACTIVATED\n\nALL ROS NODES TERMINATED"
        )
    
    def ros_node_exists(self, node_name):
        result = subprocess.run(
            "source /opt/ros/jazzy/setup.bash && ros2 node list",
            shell=True,
            executable="/bin/bash",
            capture_output=True,
            text=True
        )
        return node_name in result.stdout

def main():
    root = tk.Tk()
    app = AMRSLAMGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()