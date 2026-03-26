import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import math
import random

# ================= 1. Initialization =================
robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
target_x, target_y = 8.0, 8.0

filtered_tx, filtered_ty = target_x, target_y
alpha_lpf = 0.15  
noise_enabled = False 

max_v = 2.0
Kp_w = 3.0
dt = 0.1

K_att = 1.0       
K_rep = 25.0      
safe_radius = 2.0 
obs_radius = 0.5  
vortex_weight = 0.6  # NEW: Weight of the tangential swirling force

# Generate specific obstacles to create a "narrow corridor" for testing
obstacles = [(4.0, 3.5), (4.0, 5.5), (7.0, 7.5), (2.0, 8.0), (8.0, 2.0)]
path_x, path_y = [], []

# ================= 2. Callbacks =================
def on_click(event):
    global target_x, target_y
    if event.xdata is not None and event.ydata is not None:
        target_x, target_y = event.xdata, event.ydata
        print(f"[*] New Target: ({target_x:.1f}, {target_y:.1f})")

def on_key(event):
    global noise_enabled
    if event.key == 'n':
        noise_enabled = not noise_enabled
        state = "ON" if noise_enabled else "OFF"
        print(f"[!] Sensor Noise Simulation: {state}")

# ================= 3. Plot Setup =================
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.mpl_connect('button_press_event', on_click)
fig.canvas.mpl_connect('key_press_event', on_key)

print("\n" + "="*40)
print("Advanced APF with Vortex Field Initialized")
print("L-Click: Move UWB Target")
print("Press 'n': Toggle Sensor Noise")
print("="*40 + "\n")

# ================= 4. Main Loop =================
while True:
    if not plt.fignum_exists(fig.number): break
    
    # --- Sensor Noise & LPF Layer ---
    if noise_enabled:
        noisy_tx = target_x + random.gauss(0, 0.4)
        noisy_ty = target_y + random.gauss(0, 0.4)
        if random.random() < 0.03:
            noisy_tx += random.choice([-3.0, 3.0])
            noisy_ty += random.choice([-3.0, 3.0])
    else:
        noisy_tx, noisy_ty = target_x, target_y
        
    filtered_tx = alpha_lpf * noisy_tx + (1 - alpha_lpf) * filtered_tx
    filtered_ty = alpha_lpf * noisy_ty + (1 - alpha_lpf) * filtered_ty

    # --- Attractive Force ---
    dx_target = filtered_tx - robot_x
    dy_target = filtered_ty - robot_y
    dist_target = math.hypot(dx_target, dy_target)
    
    if dist_target > 0.1:
        F_x = K_att * dx_target / dist_target
        F_y = K_att * dy_target / dist_target
    else:
        F_x, F_y = 0.0, 0.0
        
    # --- Repulsive & Vortex Force (The Magic Fix) ---
    for ox, oy in obstacles:
        dx_obs = robot_x - ox
        dy_obs = robot_y - oy
        dist_obs = math.hypot(dx_obs, dy_obs)
        
        if 0 < dist_obs < safe_radius:
            rep_mag = K_rep * (1.0/dist_obs - 1.0/safe_radius) / (dist_obs**2)
            
            # Standard outward repulsive force vectors
            F_rep_x = rep_mag * (dx_obs / dist_obs)
            F_rep_y = rep_mag * (dy_obs / dist_obs)
            
            # Tangential vortex force vectors (rotated 90 degrees)
            F_vortex_x = -F_rep_y
            F_vortex_y = F_rep_x
            
            # Blend them together
            F_x += F_rep_x + (vortex_weight * F_vortex_x)
            F_y += F_rep_y + (vortex_weight * F_vortex_y)
            
    # --- Kinematics & P-Controller ---
    if dist_target > 0.1:
        desired_theta = math.atan2(F_y, F_x)
        angle_error = (desired_theta - robot_theta + math.pi) % (2 * math.pi) - math.pi
        w = Kp_w * angle_error
        v = max_v if abs(angle_error) < 1.0 else 0.5 
    else:
        v, w = 0.0, 0.0
        
    robot_theta += w * dt
    robot_x += v * math.cos(robot_theta) * dt
    robot_y += v * math.sin(robot_theta) * dt
    path_x.append(robot_x)
    path_y.append(robot_y)
    
    # --- Rendering ---
    ax.cla()
    ax.set_xlim(-2, 12)
    ax.set_ylim(-2, 12)
    ax.grid(True, linestyle='--', alpha=0.6)
    
    title_str = "APF + Vortex Field + LPF Tracking\n"
    if noise_enabled:
        ax.set_title(title_str + "Noise: ON", color='red', fontweight='bold')
        ax.plot(noisy_tx, noisy_ty, 'x', color='gray', markersize=10, label='Raw Data')
        ax.plot(filtered_tx, filtered_ty, 'g+', markersize=12, linewidth=2, label='Filtered Target')
    else:
        ax.set_title(title_str + "Noise: OFF", color='black')
        
    ax.plot(target_x, target_y, 'r*', markersize=15, label='UWB Target')
    
    for i, (ox, oy) in enumerate(obstacles):
        danger_zone = patches.Circle((ox, oy), safe_radius, color='red', alpha=0.1)
        ax.add_patch(danger_zone)
        obs = patches.Circle((ox, oy), obs_radius, color='dimgray')
        ax.add_patch(obs)
        if i == 0: 
            danger_zone.set_label('APF + Vortex Field')
            obs.set_label('Obstacle')
            
    ax.plot(path_x, path_y, 'b--', alpha=0.5, label='Trajectory')
    ax.arrow(robot_x, robot_y, 0.5*math.cos(robot_theta), 0.5*math.sin(robot_theta),
             head_width=0.4, head_length=0.4, fc='g', ec='g')
    
    ax.legend(loc='upper right', fontsize=8)
    plt.pause(0.01)