import tkinter as tk
from tkinter import simpledialog
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class ScaleCalculator:
    def __init__(self, master):
        self.master = master
        master.title("Scale Calculator")

        # Başlangıç ve mevcut pozisyonlar
        self.initial_pose = None
        self.current_pose = None

        # GUI elemanları
        self.direction_label = tk.Label(master, text="Select the direction of movement:")
        self.direction_label.pack()

        self.directions = ["Forward", "Backward", "Left", "Right", "Up", "Down"]
        self.direction_var = tk.StringVar(master)
        self.direction_var.set(self.directions[0])  # default value
        self.direction_menu = tk.OptionMenu(master, self.direction_var, *self.directions)
        self.direction_menu.pack()

        self.distance_label = tk.Label(master, text="Enter the actual distance moved (in meters):")
        self.distance_label.pack()

        self.distance_entry = tk.Entry(master)
        self.distance_entry.pack()

        self.calculate_button = tk.Button(master, text="Calculate Scale", command=self.calculate_scale)
        self.calculate_button.pack()

        self.result_label = tk.Label(master, text="")
        self.result_label.pack()

        # ROS subscriber
        rospy.init_node('scale_calculator_node')
        rospy.Subscriber("/orb_slam3/camera_pose", PoseStamped, self.update_pose)

    def update_pose(self, msg):
        pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        if self.initial_pose is None:
            self.initial_pose = pose
        self.current_pose = pose

    def calculate_scale(self):
        actual_distance = float(self.distance_entry.get())
        if self.current_pose is not None and self.initial_pose is not None:
            estimated_distance = np.linalg.norm(self.current_pose - self.initial_pose)
            scale = actual_distance / estimated_distance if estimated_distance != 0 else "Undefined"
            self.result_label.config(text=f"Calculated Scale for {self.direction_var.get()}: {scale}")
        else:
            self.result_label.config(text="Waiting for pose data...")

if __name__ == '__main__':
    root = tk.Tk()
    my_gui = ScaleCalculator(root)
    root.mainloop()

