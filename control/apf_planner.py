import math

class APFPlanner:
    def __init__(self, config):
        """Initialize the APF planner with parameters from the configuration"""
        self.L = config['robot']['uwb_baseline']
        self.max_v = config['robot']['max_v']
        self.safe_radius = config['apf']['safe_radius']
        self.k_att = config['apf']['k_att']
        self.k_rep = config['apf']['k_rep']
        self.vortex_weight = config['apf']['vortex_weight']
        self.kp_w = config['apf']['kp_w']

        # [新增] PD 控制与抢救逻辑所需的状态变量
        self.kd_w = config['apf'].get('kd_w', 1.5)
        self.prev_error_w = 0.0
        self.prev_dist_target = 2.0
        self.rescue_turning = False
        self.rescue_frames = 0

    def compute_command(self, uwb_dl, uwb_dr, sonar_dl, sonar_dm, sonar_dr):
        """
        Input sensor data and output velocity commands + force breakdown for visualization.

        Returns:
            v, w             : linear (m/s) and angular (rad/s) velocity
            target_x, target_y : target position in robot-local frame
            force_info (dict): {
                'F_att'   : (fx, fy),
                'F_rep_l' : (fx, fy),
                'F_rep_m' : (fx, fy),
                'F_rep_r' : (fx, fy),
                'F_total' : (fx, fy),
            }
        """
        # --- 1. Target position (robot-local frame) from UWB geometry ---
        diff = uwb_dr - uwb_dl
        diff = max(min(diff, self.L * 0.99), -self.L * 0.99)
        uwb_dr_adj = uwb_dl + diff
        
        target_y = (uwb_dr_adj**2 - uwb_dl**2) / (2 * self.L)
        val_for_sqrt = uwb_dl**2 - (target_y - self.L/2)**2
        target_x = math.sqrt(max(val_for_sqrt, 0.001))
        
        dist_target = math.hypot(target_x, target_y)

        # 基于刚算出的 target_x 和 target_y
        F_att_x = self.k_att * target_x
        F_att_y = self.k_att * target_y

        # 2. repulsion forces from sonar (in robot-local frame)
        F_rep_x = 0.0
        F_rep_y = 0.0

        # 左超声波 (不仅往右推，还给一个向前的引导力)
        if sonar_dl < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dl - 1.0/self.safe_radius) / (sonar_dl**2)
            F_rep_x += rep_mag * 0.5  # [新增] 侧向障碍也给一点前向推力，防止卡死
            F_rep_y -= rep_mag * (self.vortex_weight + 0.2) # 加大涡旋权重

        # 右超声波
        if sonar_dr < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dr - 1.0/self.safe_radius) / (sonar_dr**2)
            F_rep_x += rep_mag * 0.5  # [新增] 同理
            F_rep_y += rep_mag * (self.vortex_weight + 0.2)

        # 中间超声波 (结合 target_y 实现智能分流！)
        if sonar_dm < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dm - 1.0/self.safe_radius) / (sonar_dm**2)
            direction = 1.0 if target_y >= 0 else -1.0
            
            F_rep_x -= rep_mag
            F_rep_y += rep_mag * self.vortex_weight * direction

        # --- 3. Total force ---
        F_total_x = F_att_x + F_rep_x
        F_total_y = F_att_y + F_rep_y

        # --- 4. Velocity commands ---
        # 抢救逻辑触发判断：
        # 如果人之前离车很近 (< 0.6m)，并且距离突然开始变大(>0.1m)，
        # 极大概率是人由于车身延迟已经走到了小车后方（陷入镜像死区）
        if dist_target > self.prev_dist_target + 0.1 and self.prev_dist_target < 0.6:
            self.rescue_turning = True
            self.rescue_frames = 10  # 强制掉头状态持续 10 帧 (约 1 秒)

        self.prev_dist_target = dist_target

        if self.rescue_turning:
            # 执行抢救：停住平移，强制大角度原地甩头找人
            v = 0.0
            w = 2.0  # 向左猛打方向盘（根据你基站习惯可改为负数）
            self.rescue_frames -= 1
            if self.rescue_frames <= 0:
                self.rescue_turning = False
        
        elif dist_target > 0.2:
            # 正常跟随：引入 PD 转向控制对抗通讯延迟
            error_w = math.atan2(F_total_y, F_total_x)
    
            # 1. 转向优先级逻辑：如果角度偏差大于 30 度 (约 0.5 rad)，大幅削减线速度
            if abs(error_w) > 0.5:
                v_scale = 0.1  # 几乎原地转
            else:
                v_scale = 1.0  # 正常行驶
                
            v = self.max_v * v_scale * max(0.0, (1.0 - abs(error_w)/math.pi))
            d_error = error_w - self.prev_error_w
            
            # 2. 更加激进的 PD 转向 (增加微分项抗过冲)
            w = (self.kp_w * error_w) + (self.kd_w * d_error)
            
            # 预测目标的横移趋势，极大提升转向“跟手感”
            self.prev_error_w = error_w
            
            # 为了防止在打大方向时车速过快冲出去，限制转弯时的线速度
            if F_total_x > 0:
                # 角度越大，线速度打折越多
                v = self.max_v * max(0.2, (1.0 - abs(error_w)/math.pi)) 
            else:
                v = 0.0
        else:
            v, w = 0.0, 0.0

        force_info = {
            'F_att'   : (F_att_x,   F_att_y),
            'F_rep_l' : (F_rep_x, F_rep_y),  # 左侧超声波斥力
            'F_rep_m' : (F_rep_x, F_rep_y),  # 中间超声波斥力
            'F_rep_r' : (F_rep_x, F_rep_y),  # 右侧超声波斥力
            'F_total' : (F_total_x, F_total_y),
        }

        return v, w, target_x, target_y, force_info