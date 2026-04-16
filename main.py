import time
import math
import yaml
import sys
import matplotlib
matplotlib.use('TkAgg')          # 使用 TkAgg 后端；如在无头环境可改成 'Agg'
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyArrowPatch
 
from sensors.sonar import SonarArray
from sensors.uwb import DualUWBManager
from control.apf_planner import APFPlanner
from hardware.motor_driver import MotorController
 
 
# ─────────────────────────────────────────────
#  配置加载
# ─────────────────────────────────────────────
def load_config(path="config.yaml"):
    try:
        with open(path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"[!] Failed to load config: {e}")
        sys.exit(1)
 
 
# ─────────────────────────────────────────────
#  可视化辅助：在 local-frame 图上画受力
# ─────────────────────────────────────────────
ARROW_SCALE = 0.5          # 力向量长度缩放（视觉）
SONAR_POSITIONS = {        # 三个超声波在机器人 local 坐标系中的安装位置 (x, y)
    'left':   (0.15,  0.15),
    'mid':    (0.20,  0.00),
    'right':  (0.15, -0.15),
}
 
def _draw_arrow(ax, x, y, dx, dy, color, label=None, lw=2, alpha=0.9):
    """画一条从 (x,y) 出发的力向量箭头。"""
    if abs(dx) < 1e-6 and abs(dy) < 1e-6:
        return
    ax.annotate(
        "", xy=(x + dx * ARROW_SCALE, y + dy * ARROW_SCALE), xytext=(x, y),
        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw),
        alpha=alpha, label=label
    )
 
 
def draw_local_frame(ax_local, target_x, target_y, force_info, sonar_dl, sonar_dm, sonar_dr, safe_radius):
    """
    左侧子图：机器人局部坐标系视角
      · 小车永远在原点，朝向 +X 轴
      · 显示 UWB 解算出的目标点
      · 显示引力 / 斥力 / 合力箭头
      · 用扇形示意每个超声波的障碍距离
    """
    ax_local.cla()
    ax_local.set_aspect('equal')
    ax_local.set_xlim(-0.5, 3.5)
    ax_local.set_ylim(-2.0, 2.0)
    ax_local.set_facecolor('#0d1117')
    ax_local.tick_params(colors='#8b949e')
    for spine in ax_local.spines.values():
        spine.set_edgecolor('#30363d')
    ax_local.set_title("Robot Local Frame  (APF Forces)", color='#e6edf3', fontsize=10, pad=6)
    ax_local.set_xlabel("X forward (m)", color='#8b949e', fontsize=8)
    ax_local.set_ylabel("Y left (m)",    color='#8b949e', fontsize=8)
    ax_local.axhline(0, color='#30363d', lw=0.8)
    ax_local.axvline(0, color='#30363d', lw=0.8)
    ax_local.grid(True, color='#21262d', lw=0.5)
 
    # ── 机器人本体 ──
    robot_body = plt.Circle((0, 0), 0.18, color='#238636', zorder=5)
    ax_local.add_patch(robot_body)
    ax_local.annotate("", xy=(0.35, 0), xytext=(0, 0),
                      arrowprops=dict(arrowstyle="-|>", color='#f0f6fc', lw=2), zorder=6)
 
    # ── 超声波障碍圆弧（用圆表示探测到的障碍距离） ──
    sonar_colors = {'left': '#f85149', 'mid': '#ff7b72', 'right': '#f85149'}
    sonar_dists  = {'left': sonar_dl,  'mid': sonar_dm,  'right': sonar_dr}
    for name, (sx, sy) in SONAR_POSITIONS.items():
        d = sonar_dists[name]
        color = sonar_colors[name]
        alpha = 0.7 if d < safe_radius else 0.2
        # 画一个小圆表示障碍物位置
        obs_x = sx + d
        obs_circle = plt.Circle((obs_x, sy), 0.06, color=color, alpha=alpha, zorder=4)
        ax_local.add_patch(obs_circle)
        # 虚线从传感器到障碍
        ax_local.plot([sx, obs_x], [sy, sy], '--', color=color, alpha=alpha * 0.8, lw=1.0)
        # 超声波安装点
        ax_local.plot(sx, sy, 'o', color=color, markersize=4, alpha=0.9, zorder=5)
 
    # ── 目标点（UWB 解算） ──
    ax_local.plot(target_x, target_y, '*', color='#f0a500', markersize=14,
                  zorder=7, label=f'UWB Target ({target_x:.2f}, {target_y:.2f})')
 
    # ── 力向量 ──
    origin = (0.0, 0.0)
    # 引力（绿色）
    _draw_arrow(ax_local, *origin, *force_info['F_att'],   color='#3fb950', lw=2.5)
    # 斥力：左、中、右（红色系）
    _draw_arrow(ax_local, *SONAR_POSITIONS['left'],  *force_info['F_rep_l'], color='#f85149', lw=1.8)
    _draw_arrow(ax_local, *SONAR_POSITIONS['mid'],   *force_info['F_rep_m'], color='#ff7b72', lw=1.8)
    _draw_arrow(ax_local, *SONAR_POSITIONS['right'], *force_info['F_rep_r'], color='#f85149', lw=1.8)
    # 合力（白色、最粗）
    _draw_arrow(ax_local, *origin, *force_info['F_total'],  color='#f0f6fc', lw=3.0)
 
    # ── 图例 ──
    legend_patches = [
        mpatches.Patch(color='#3fb950', label='Attraction (F_att)'),
        mpatches.Patch(color='#f85149', label='Repulsion (F_rep)'),
        mpatches.Patch(color='#f0f6fc', label='Total force (F_total)'),
        mpatches.Patch(color='#f0a500', label='UWB Target'),
    ]
    ax_local.legend(handles=legend_patches, loc='upper right',
                    fontsize=7, facecolor='#161b22', edgecolor='#30363d',
                    labelcolor='#c9d1d9')
 
 
def draw_global_map(ax_global, robot_x, robot_y, robot_theta,
                    path_x, path_y, target_gx, target_gy,
                    v, w, force_info):
    """
    右侧子图：全局地图
      · 小车轨迹
      · 当前位置 + 朝向箭头
      · UWB 目标（全局坐标）
      · 合力方向（转到全局系后）
    """
    ax_global.cla()
    ax_global.set_aspect('equal')
    view = 4.0
    ax_global.set_xlim(robot_x - view, robot_x + view)
    ax_global.set_ylim(robot_y - view, robot_y + view)
    ax_global.set_facecolor('#0d1117')
    ax_global.tick_params(colors='#8b949e')
    for spine in ax_global.spines.values():
        spine.set_edgecolor('#30363d')
    ax_global.set_title("Global Map  (Robot Trajectory)", color='#e6edf3', fontsize=10, pad=6)
    ax_global.set_xlabel("X (m)", color='#8b949e', fontsize=8)
    ax_global.set_ylabel("Y (m)", color='#8b949e', fontsize=8)
    ax_global.grid(True, color='#21262d', lw=0.5)
 
    # ── 轨迹 ──
    if len(path_x) > 1:
        ax_global.plot(path_x, path_y, '-', color='#1f6feb', alpha=0.5, lw=1.5, label='Path')
 
    # ── 目标点 ──
    ax_global.plot(target_gx, target_gy, '*', color='#f0a500', markersize=14,
                   zorder=7, label='UWB Target')
    # 目标轨迹（淡）
    ax_global.plot(target_gx, target_gy, 'o', color='#f0a500',
                   alpha=0.2, markersize=6, zorder=3)
 
    # ── 机器人 ──
    robot_circle = plt.Circle((robot_x, robot_y), 0.18, color='#238636', zorder=5)
    ax_global.add_patch(robot_circle)
    ax_global.annotate(
        "", xy=(robot_x + 0.4*math.cos(robot_theta),
                robot_y + 0.4*math.sin(robot_theta)),
        xytext=(robot_x, robot_y),
        arrowprops=dict(arrowstyle="-|>", color='#f0f6fc', lw=2), zorder=6
    )
 
    # ── 合力方向（全局系） ──
    ftx, fty = force_info['F_total']
    ftx_g = ftx * math.cos(robot_theta) - fty * math.sin(robot_theta)
    fty_g = ftx * math.sin(robot_theta) + fty * math.cos(robot_theta)
    _draw_arrow(ax_global, robot_x, robot_y, ftx_g, fty_g, color='#f0f6fc', lw=2.5)
 
    # ── 状态文字 ──
    status = (f"v = {v:+.2f} m/s   w = {w:+.2f} rad/s\n"
              f"θ = {math.degrees(robot_theta):+.1f}°   "
              f"pos = ({robot_x:.2f}, {robot_y:.2f})")
    ax_global.text(robot_x - view + 0.15, robot_y + view - 0.25, status,
                   color='#c9d1d9', fontsize=8, family='monospace',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='#161b22',
                             edgecolor='#30363d', alpha=0.85))
 
    ax_global.legend(loc='lower right', fontsize=7,
                     facecolor='#161b22', edgecolor='#30363d', labelcolor='#c9d1d9')
 
 
# ─────────────────────────────────────────────
#  主程序
# ─────────────────────────────────────────────
def main():
    print("=" * 50)
    print("  APF-Based UWB Follow Robot  (with Sim View)")
    print("=" * 50)
 
    config = load_config()
    STOP_THRESHOLD = 0.5
    DT = 0.1
 
    # ── 初始化模块 ──
    sonar   = SonarArray(config)
    uwb     = DualUWBManager(config)
    planner = APFPlanner(config)
    chassis = MotorController()
 
    # ── 虚拟位姿（仿真积分用） ──
    sim_x, sim_y, sim_theta = 0.0, 0.0, 0.0
    path_x, path_y = [0.0], [0.0]
    PATH_MAX = 300
    '''
    # ── 建立 matplotlib 窗口 ──
    plt.style.use('dark_background')
    fig, (ax_local, ax_global) = plt.subplots(1, 2, figsize=(14, 6))
    fig.patch.set_facecolor('#0d1117')
    fig.suptitle("UWB Follow Robot — APF Visualizer", color='#e6edf3',
                 fontsize=13, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.ion()
    plt.show()
    '''
    print("\n[*] Entering main control loop... (Ctrl+C to exit)\n")
 
    try:
        while True:
            loop_start = time.time()
 
            # ── A. 读传感器 ──
            sonar_dl, sonar_dm, sonar_dr = sonar.get_distances()
            uwb_dl, uwb_dr = uwb.get_distances()

            # ── B. APF 规划（返回受力信息） ──
            v, w, tgt_x, tgt_y, force_info = planner.compute_command(
                uwb_dl, uwb_dr, sonar_dl, sonar_dm, sonar_dr
            )

            if min(uwb_dl, uwb_dr) < 0.5:
                v *= 0.5  # UWB 距离过近时减速，增加稳定性
 
            # ── 独立避障逻辑 (Behavioral Obstacle Avoidance) ──
            AVOID_THRESHOLD = 0.6  # 开始产生切向避障响应的距离阈值
            AVOID_GAIN = 1.0       # 避障切向力的强度系数（可根据实车表现调整）
            
            min_dist = min(sonar_dl, sonar_dm, sonar_dr)
            
            w_avoid = 0.0  # 初始避障角速度补偿为0
            
            # 如果进入避障范围，计算“切向逃逸”角速度
            if min_dist < AVOID_THRESHOLD:
                # 1. 左侧有障碍：往右躲（负角速度）
                if sonar_dl < AVOID_THRESHOLD:
                    w_avoid -= AVOID_GAIN * (AVOID_THRESHOLD - sonar_dl)
                
                # 2. 右侧有障碍：往左躲（正角速度）
                if sonar_dr < AVOID_THRESHOLD:
                    w_avoid += AVOID_GAIN * (AVOID_THRESHOLD - sonar_dr)
                
                # 3. 正前方有障碍：根据 APF 原始的转向意图(w的符号)决定往哪边绕，优先顺着目标方向
                if sonar_dm < AVOID_THRESHOLD:
                    target_dir = 1.0 if w >= 0 else -1.0
                    w_avoid += target_dir * AVOID_GAIN * (AVOID_THRESHOLD - sonar_dm)

            # 将避障的切向反应叠加到 APF 的追踪意图上
            final_w = w + w_avoid

            # ── C. 执行运动 ──
            if min_dist <= STOP_THRESHOLD:
                # 触发极度危险距离：切断线速度 v，原地旋转自救
                chassis.stop_all()          # 先刹车一帧，确保车身静止
                chassis.send_cmd_vel(0, final_w)  # 再下发纯旋转指令
                print(f"[Obstacle] STOP! min={min_dist:.2f}m | w={final_w:.2f} (w_base:{w:.2f}, w_avoid:{w_avoid:.2f})")
            else:
                # 正常安全行驶，或轻微切向避障中
                chassis.send_cmd_vel(v, final_w)
            '''
            # ── D. 仿真位姿积分 ──
            sim_theta += w * DT
            sim_x     += v * math.cos(sim_theta) * DT
            sim_y     += v * math.sin(sim_theta) * DT
            path_x.append(sim_x)
            path_y.append(sim_y)
            if len(path_x) > PATH_MAX:
                path_x.pop(0); path_y.pop(0)
 
            # ── 目标点转全局坐标 ──
            tgt_gx = sim_x + tgt_x*math.cos(sim_theta) - tgt_y*math.sin(sim_theta)
            tgt_gy = sim_y + tgt_x*math.sin(sim_theta) + tgt_y*math.cos(sim_theta)
 
            # ── E. 可视化更新 ──
            
            draw_local_frame(ax_local, tgt_x, tgt_y, force_info,
                             sonar_dl, sonar_dm, sonar_dr,
                             config['apf']['safe_radius'])
            draw_global_map(ax_global, sim_x, sim_y, sim_theta,
                            path_x, path_y, tgt_gx, tgt_gy,
                            v, w, force_info)
            plt.pause(0.001)
 
            # ── F. 终端监控 ──
            fa  = force_info['F_att']
            ft  = force_info['F_total']
            print(
                f"[Mon] UWB L:{uwb_dl:.2f} R:{uwb_dr:.2f} | "
                f"Tgt({tgt_x:.2f},{tgt_y:.2f}) | "
                f"F_att({fa[0]:.2f},{fa[1]:.2f}) "
                f"F_tot({ft[0]:.2f},{ft[1]:.2f}) | "
                f"v={v:.2f} w={w:.2f}"
            )
            '''            
 
            # ── G. 频率控制 ~10Hz ──
            time.sleep(max(DT - (time.time() - loop_start), 0))
 
    except KeyboardInterrupt:
        print("\n[*] Interrupt received — shutting down...")
        chassis.send_cmd_vel(0, 0)
        plt.ioff()
        plt.close('all')
        print("[*] System shut down safely.")
 
 
if __name__ == "__main__":
    main()
