import math

class APFPlanner:
    def __init__(self, config):
        # --- 机器人硬件参数 ---
        self.L = config['robot']['uwb_baseline']
        self.max_v = config['robot']['max_v']
        self.max_w = 3.0  # 允许的最大角速度 (rad/s)，可根据底盘极限调大
        
        # --- APF 核心参数 ---
        self.safe_radius = config['apf']['safe_radius']
        self.k_att = config['apf']['k_att']
        self.k_rep = config['apf']['k_rep']
        self.vortex_weight = config['apf']['vortex_weight']
        
        # --- 转向控制参数 (PD) ---
        self.kp_w = config['apf']['kp_w']
        self.kd_w = config['apf'].get('kd_w', 1.5)  # 默认给个1.5的微分系数
        self.prev_error_w = 0.0
        
        # --- 状态记录 ---
        self.prev_target_y = 0.0

    def compute_command(self, uwb_dl, uwb_dr, sonar_dl, sonar_dm, sonar_dr):
        """
        核心解算循环：
        1. 寻找目标 -> 2. 计算引力 -> 3. 计算斥力(目标导向) -> 4. 向量合成 -> 5. 运动学输出
        """
        
        # ==========================================
        # 1. 目标定位 (带有奇点保护)
        # ==========================================
        # 限制基站差值，防止出现物理上不可能的三角形成立条件
        diff = max(min(uwb_dr - uwb_dl, self.L * 0.99), -self.L * 0.99)
        uwb_dr_adj = uwb_dl + diff
        
        # 使用余弦定理推导目标坐标
        target_y = (uwb_dr_adj**2 - uwb_dl**2) / (2 * self.L)
        val_for_sqrt = uwb_dl**2 - (target_y - self.L/2)**2
        target_x = math.sqrt(max(val_for_sqrt, 0.001)) # 永远假设目标在车头前方(镜像约束)
        
        # 平滑处理：防止极短时间内的目标 Y 轴跳变
        target_y = 0.7 * target_y + 0.3 * self.prev_target_y
        self.prev_target_y = target_y
        
        dist_target = math.hypot(target_x, target_y)

        # ==========================================
        # 2. 引力计算 (目标绝对优先)
        # ==========================================
        # 距离越远，引力越大。为了防止跑太远时引力爆炸，可以加一个软饱和
        att_mag = self.k_att * min(dist_target, 3.0) 
        angle_to_target = math.atan2(target_y, target_x)
        
        F_att_x = att_mag * math.cos(angle_to_target)
        F_att_y = att_mag * math.sin(angle_to_target)

        # ==========================================
        # 3. 斥力计算 (目标导向的动态涡旋场)
        # ==========================================
        F_rep_x = 0.0
        F_rep_y = 0.0

        def calc_rep_mag(dist):
            """计算基础斥力标量"""
            if dist >= self.safe_radius or dist <= 0.05: # 小于5cm视为盲区噪点
                return 0.0
            return self.k_rep * (1.0/dist - 1.0/self.safe_radius) / (dist**2)

        # 3.1 左超声波 (障碍物在左)
        mag_l = calc_rep_mag(sonar_dl)
        if mag_l > 0:
            F_rep_x += mag_l * 0.2  # 【Feature】给一点前向推力，防止在墙边死锁
            F_rep_y -= mag_l * self.vortex_weight # 往右推

        # 3.2 右超声波 (障碍物在右)
        mag_r = calc_rep_mag(sonar_dr)
        if mag_r > 0:
            F_rep_x += mag_r * 0.2  # 【Feature】给一点前向推力
            F_rep_y += mag_r * self.vortex_weight # 往左推

        # 3.3 中间超声波 (正前方障碍物)
        mag_m = calc_rep_mag(sonar_dm)
        if mag_m > 0:
            # 【Core 1】目标在哪侧，就顺势往哪侧产生涡旋力！
            turn_direction = 1.0 if target_y >= 0 else -1.0 
            
            F_rep_x -= mag_m # 强烈向后推
            F_rep_y += mag_m * self.vortex_weight * turn_direction # 极其丝滑的顺势绕行

        # ==========================================
        # 4. 向量合成
        # ==========================================
        F_total_x = F_att_x + F_rep_x
        F_total_y = F_att_y + F_rep_y

        # ==========================================
        # 5. 运动学解算与灵敏度控制
        # ==========================================
        # 计算合力方向 (即小车期望的朝向)
        error_w = math.atan2(F_total_y, F_total_x)
        
        # 5.1 【Core 2】暴躁且精准的转向 PD 控制
        d_error = error_w - self.prev_error_w
        self.prev_error_w = error_w
        
        w = (self.kp_w * error_w) + (self.kd_w * d_error)
        w = max(min(w, self.max_w), -self.max_w) # 限幅保护
        
        # 【Feature】死区补偿：如果算出的转向力太小，强制放大以克服电机静摩擦
        MIN_W = 0.4
        if 0 < abs(w) < MIN_W:
            w = MIN_W if w > 0 else -MIN_W

        # 5.2 【Core 2】大角度转向优先：切断线速度
        if dist_target < 0.2:
            # 靠得足够近了，直接刹车
            v = 0.0
            w = 0.0
        else:
            # 角度偏离超过 45 度 (约 0.78 rad) 时，小车几乎不往前走，全力原地转向
            angle_penalty = max(0.0, 1.0 - abs(error_w) / 0.78)
            
            # 如果合力向后 (比如离墙太近被弹开)，允许小车缓慢倒车
            if F_total_x < 0:
                v = -0.3 * self.max_v
            else:
                v = self.max_v * angle_penalty

        return v, w, F_total_x, F_total_y, target_x, target_y