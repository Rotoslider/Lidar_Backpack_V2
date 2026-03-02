import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
from smbus2 import SMBus
import os
import signal
import socket
import time

class MotorController:
    def __init__(self):
        self.dev_addr = 0x28  # Device address if switch set to off
        self.bus = SMBus(17)   # I2C bus number may need changing

    def miranda_write(self, address, command, tx_data):
        try:
            self.bus.write_i2c_block_data(address, command, tx_data)
        except Exception as e:
            print(f"Failed to write to device: {e}")

    def start_motor(self, rpm=20):
        # Calculate motor speed counts for 20 RPM
        msb, lsb = self.rpm_to_counts(rpm)
        self.miranda_write(self.dev_addr, 0x07, [msb, lsb])

    def stop_motor(self):
        # Stop motor command
        self.miranda_write(self.dev_addr, 0x07, [0x00, 0x00])

    def rpm_to_counts(self, rpm):
        deg_per_sec = rpm * 6
        motor_counts = int(deg_per_sec * 91.01944444444445)
        msb = (motor_counts >> 8) & 0xFF
        lsb = motor_counts & 0xFF
        return msb, lsb

class ScannerControlApp:
    def __init__(self, master):
        self.master = master
        master.title("3D Scanner Control")
        self.motor_controller = MotorController()

        self.processes = []

        self.start_button = ttk.Button(master, text="Start Scanning", command=self.start_scanning)
        self.start_button.grid(row=0, column=0, padx=10, pady=10)

        self.stop_button = ttk.Button(master, text="Stop Scanning", command=self.stop_all)
        self.stop_button.grid(row=0, column=1, padx=10, pady=10)

        self.exit_button = ttk.Button(master, text="Exit", command=self.exit_app)
        self.exit_button.grid(row=1, column=0, columnspan=2, padx=10, pady=10)


    def set_lidar_standby(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
                # Connect to LiDAR
                sock.connect(("169.254.201.29", 7501))
                # Send the command to switch to standby mode
                sock.sendall(b"set_config_param operating_mode STANDBY\nreinitialize\n")
                # Wait a bit to ensure commands are processed
                time.sleep(0.1)
                # Close the connection
                sock.close()
        except Exception as e:
            print(f"Error sending standby command to LiDAR: {e}")
            self.show_message("Failed to set LiDAR to standby mode!")
            return  # Ensure to exit if there's an error
        self.show_message("LiDAR set to standby mode successfully!")

    def start_scanning(self):
        self.show_message("Getting Ready, please wait...", 100)
        self.master.after(100, self.start_lidar)  # Start LiDAR after 40 seconds, allows time for it to spin up

    def start_lidar(self):
        ouster_command = "bash -c 'source /opt/ros/noetic/setup.bash && source /home/lidar/catkin_ws/devel/setup.bash && roslaunch ouster_ros sensor.launch viz:=false sensor_hostname:=169.254.201.29'"
        proc = subprocess.Popen(ouster_command, shell=True, preexec_fn=os.setsid)
        self.processes.append(proc)
        self.show_message("Starting LIDAR... Wait 40 seconds till scan starts", 40000)
        self.master.after(40000, self.start_slam)

    def start_slam(self):
        slam_command = "bash -c 'source /opt/ros/noetic/setup.bash && source /home/lidar/catkin_ws/devel/setup.bash && roslaunch fast_lio mapping_ouster32.launch sensor_hostname:=169.254.201.29'"
        proc = subprocess.Popen(slam_command, shell=True, preexec_fn=os.setsid)
        self.processes.append(proc)
        self.master.after(1000, self.start_recording)

    def start_recording(self):
        bag_command = "bash -c 'source /opt/ros/noetic/setup.bash && source /home/lidar/catkin_ws/devel/setup.bash && rosbag record -o /home/lidar/pointclouds/OS0_32 /ouster/imu_packets /ouster/lidar_packets'"
        proc = subprocess.Popen(bag_command, shell=True, preexec_fn=os.setsid)
        self.processes.append(proc)
        self.show_message("Recording Now, press 'Stop Scanning' when finished.", 20000)
        self.master.after(10000, lambda: self.motor_controller.start_motor(20))  # Starts motor spinning after bag starts recording

    def stop_all(self):
        self.set_lidar_standby()
        for proc in self.processes:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)  # Send the signal to all the process groups
        self.processes.clear()
        self.motor_controller.stop_motor()
        self.show_message("Scanning stopped successfully!")

    def show_message(self, message, delay=3000):
        popup = tk.Toplevel(self.master)
        popup.title("Notification")
        popup.overrideredirect(True)  # Removes the window decorations (title bar, etc.)
        popup.geometry("350x30+{}+{}".format(self.master.winfo_x() + 0, self.master.winfo_y() + 100))  # Adjust size and position

        label = ttk.Label(popup, text=message, background="white", relief="solid", borderwidth=2, justify="center", anchor="center")
        label.pack(ipadx=10, ipady=10, expand=True, fill="both")  # Add internal padding around the text

        #label = ttk.Label(popup, text=message)
        #label.pack(side="top", fill="x", pady=10)
        self.master.after(delay, popup.destroy)

    def exit_app(self):
        self.stop_all()
        self.master.quit()

root = tk.Tk()
app = ScannerControlApp(root)
root.mainloop()

