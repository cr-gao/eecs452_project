import math
import matplotlib.pyplot as plt

class VirtualChassis:
    def __init__(self):
        """Initialize the virtual chassis for visualization"""
        print("[*] Initialize virtual chassis...")
        self.x = 0.0          # Global X coordinate
        self.y = 0.0          # Global Y coordinate
        self.theta = 0.0      # Global orientation (radians)
        self.path_x = []
        self.path_y = []
        
        self.dt = 0.1         # Simulation time step (s)

        # Dynamically create a matplotlib figure for visualization
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.fig.canvas.manager.set_window_title("UWB + Sonar HIL Test")

    def send_cmd_vel(self, v, w, target_local_x, target_local_y):
        """
        Receive velocity commands and target coordinates, update the virtual robot's state, and render the visualization.
        """
        # --- 1. Update the virtual robot's state ---
        self.theta += w * self.dt
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt

        self.path_x.append(self.x)
        self.path_y.append(self.y)
        if len(self.path_x) > 150:  # Limit the length of the path history for better visualization
            self.path_x.pop(0)
            self.path_y.pop(0)

        # --- 2. Transform target coordinates from local to global frame ---
        # UWB 算出来的是车头坐标系，需要转到全局坐标系才能画在地图上
        target_global_x = self.x + target_local_x * math.cos(self.theta) - target_local_y * math.sin(self.theta)
        target_global_y = self.y + target_local_x * math.sin(self.theta) + target_local_y * math.cos(self.theta)

        # --- 3. Render the visualization ---
        self.ax.cla()
        
        # Dynamically adjust the view to keep the robot centered
        self.ax.set_xlim(self.x - 3, self.x + 3)
        self.ax.set_ylim(self.y - 3, self.y + 3)
        self.ax.grid(True, linestyle='--', alpha=0.6)

        # Plot the robot's path
        self.ax.plot(self.path_x, self.path_y, 'b--', alpha=0.5, label='Robot Path')

        # Plot the target point calculated from UWB (red star)
        self.ax.plot(target_global_x, target_global_y, 'r*', markersize=12, label='UWB Target')

        # Plot the robot as a green circle with an arrow indicating orientation
        car_circle = plt.Circle((self.x, self.y), 0.2, color='green', fill=True)
        self.ax.add_patch(car_circle)
        self.ax.arrow(self.x, self.y, 0.4*math.cos(self.theta), 0.4*math.sin(self.theta),
                      head_width=0.15, head_length=0.15, fc='yellow', ec='black', zorder=5)

        self.ax.legend(loc='lower right', fontsize=8)
        self.ax.set_title(f"V: {v:.2f} m/s | W: {w:.2f} rad/s")
        
        # Stop and show the updated plot
        plt.pause(0.001)