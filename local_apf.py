import matplotlib.pyplot as plt
import numpy as np
import math
import time
from gpiozero import DistanceSensor

# ================= 1. 硬件传感器初始化 =================
print("[*] 初始化超声波传感器...")
try:
    # 距离限制设为 2.0 米，超过这个距离视为安全
    sonar_left = DistanceSensor(echo=24, trigger=23, max_distance=2.0)
    sonar_right = DistanceSensor(echo=27, trigger=17, max_distance=2.0)
except Exception as e:
    print(f"[!] 传感器初始化失败，请检查接线和权限: {e}")
    exit(1)

# ================= 2. 算法与运动学参数 =================
# 虚拟小车状态 (全局坐标系)
robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
path_x, path_y = [], []

# UWB模拟目标 (初始位置)
target_x, target_y = 5.0, 5.0

# 避障参数
safe_radius = 0.6       # 60cm 内开始产生斥力 (单位：米)
K_att = 1.2             # 引力系数
K_rep = 0.8             # 斥力系数
vortex_weight = 0.6     # 旋涡力权重 (绕行幅度)
max_v = 1.0             # 最大线速度 (m/s)
Kp_w = 2.0              # 转向P控制系数
dt = 0.1                # 仿真步长

# ================= 3. 模拟 UWB 接口 =================
def on_click(event):
    global target_x, target_y
    if event.xdata is not None and event.ydata is not None:
        target_x, target_y = event.xdata, event.ydata
        print(f"[*] 模拟UWB目标更新: ({target_x:.2f}, {target_y:.2f})")

# ================= 4. 可视化设置 =================
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
fig.canvas.mpl_connect('button_press_event', on_click)
print("\n" + "="*45)
print("半实物仿真启动：真实超声波 + 虚拟底盘/UWB")
print("请在图像上点击左键来移动 UWB 目标！")
print("用手或障碍物在真实传感器前晃动来测试避障！")
print("="*45 + "\n")

# ================= 5. 主循环 =================
while True:
    if not plt.fignum_exists(fig.number): 
        break
    
    # --- A. 获取真实传感器数据 (单位转换为米) ---
    d_left = sonar_left.distance
    d_right = sonar_right.distance
    
    # 打印真实数据以便你在硬件端调试
    print(f"真实超声波 -> 左: {d_left*100:.1f}cm, 右: {d_right*100:.1f}cm", end='\r')

    # --- B. 计算局部坐标系下的力 (X向前，Y向左) ---
    F_local_x = 0.0
    F_local_y = 0.0
    
    # 处理左侧超声波
    if d_left < safe_radius:
        rep_mag = K_rep * (1.0/d_left - 1.0/safe_radius) / (d_left**2)
        F_local_y -= rep_mag                    # 斥力：向右推 (-Y)
        F_local_x += rep_mag * vortex_weight    # 旋涡力：向前滑 (+X)
        
    # 处理右侧超声波
    if d_right < safe_radius:
        rep_mag = K_rep * (1.0/d_right - 1.0/safe_radius) / (d_right**2)
        F_local_y += rep_mag                    # 斥力：向左推 (+Y)
        F_local_x += rep_mag * vortex_weight    # 旋涡力：向前滑 (+X)

    # --- C. 计算UWB目标的局部引力 ---
    # 计算目标在全局坐标系下的相对向量
    dx_target = target_x - robot_x
    dy_target = target_y - robot_y
    dist_target = math.hypot(dx_target, dy_target)
    
    if dist_target > 0.1:
        # 目标在全局坐标系的角度
        global_target_angle = math.atan2(dy_target, dx_target)
        # 将目标角度转换到小车的局部坐标系 (目标相对于车头的夹角)
        local_target_angle = global_target_angle - robot_theta
        
        # 将引力分解到小车的前后(X)和左右(Y)
        F_att_x = K_att * math.cos(local_target_angle)
        F_att_y = K_att * math.sin(local_target_angle)
    else:
        F_att_x, F_att_y = 0.0, 0.0

    # --- D. 合成合力与运动学控制 ---
    F_total_x = F_att_x + F_local_x
    F_total_y = F_att_y + F_local_y

    if dist_target > 0.1:
        # 期望的局部转向角 (y/x)
        desired_w = math.atan2(F_total_y, F_total_x)
        w = Kp_w * desired_w
        
        # 如果前方受阻严重（合力X轴分量很小甚至为负），应当减速
        v = max_v if F_total_x > 0 else 0.0
    else:
        v, w = 0.0, 0.0

    # 更新虚拟小车位姿 (运动学模型)
    robot_theta += w * dt
    robot_x += v * math.cos(robot_theta) * dt
    robot_y += v * math.sin(robot_theta) * dt
    
    path_x.append(robot_x)
    path_y.append(robot_y)
    # 保持轨迹长度，防止内存爆炸
    if len(path_x) > 200: 
        path_x.pop(0)
        path_y.pop(0)

    # --- E. 渲染画面 ---
    ax.cla()
    # 动态跟随视角 (让画面始终以小车为中心，类似真实的跟随效果)
    ax.set_xlim(robot_x - 4, robot_x + 4)
    ax.set_ylim(robot_y - 4, robot_y + 4)
    ax.grid(True, linestyle='--', alpha=0.6)
    
    # 绘制目标点和轨迹
    ax.plot(target_x, target_y, 'r*', markersize=15, label='Simulated UWB Target')
    ax.plot(path_x, path_y, 'b--', alpha=0.5, label='Virtual Trajectory')
    
    # 绘制虚拟小车
    car_circle = plt.Circle((robot_x, robot_y), 0.2, color='green', fill=True, label='Virtual Robot')
    ax.add_patch(car_circle)
    # 绘制车头方向指示
    ax.arrow(robot_x, robot_y, 0.4*math.cos(robot_theta), 0.4*math.sin(robot_theta),
             head_width=0.15, head_length=0.15, fc='yellow', ec='black')
    
    # 在画面上显示真实传感器读数，方便脱离终端观察
    ax.text(robot_x - 3.8, robot_y + 3.5, f"Real Sonar L: {d_left*100:.0f}cm", fontsize=10, color='blue', bbox=dict(facecolor='white', alpha=0.7))
    ax.text(robot_x - 3.8, robot_y + 3.1, f"Real Sonar R: {d_right*100:.0f}cm", fontsize=10, color='blue', bbox=dict(facecolor='white', alpha=0.7))

    ax.legend(loc='lower right', fontsize=8)
    ax.set_title("HIL Test: Real Sensor + Virtual Kinematics")
    
    plt.pause(dt)