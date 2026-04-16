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

    def compute_command(self, uwb_dl, uwb_dr, sonar_dl, sonar_dm, sonar_dr):
        """
        极简版纯追踪逻辑：只定位，只锁头。
        """

        # --- 1. 纯粹的几何定位 (保持不变) ---
        target_y = (uwb_dr**2 - uwb_dl**2) / (2 * self.L)
        val_for_sqrt = uwb_dl**2 - (target_y - self.L/2)**2
        target_x = math.sqrt(max(val_for_sqrt, 0.001))

        # --- 2. 极简的引力与运动学输出 ---
        # 放弃归一化，使用更符合物理直觉的“弹簧模型”：距离越远，力越大
        F_att_x = self.k_att * target_x
        F_att_y = self.k_att * target_y

        # --- [新增] 3. 极简斥力与切向力 (目标方向优先 & 传感器反向优先) ---
        F_rep_x, F_rep_y = 0.0, 0.0
        F_rep_l, F_rep_m, F_rep_r = (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)

        # 辅助函数：计算基础斥力标量
        def calc_rep_mag(dist):
            if dist >= self.safe_radius or dist <= 0.05:
                return 0.0
            return self.k_rep * (1.0/dist - 1.0/self.safe_radius) / (dist**2)

        # 目标方位符号：目标在左(y>0)为1，在右(y<0)为-1
        target_dir = 1.0 if target_y >= 0 else -1.0 

        # 3.1 中间超声波 (正前方遇阻)：优先往【目标所在方向】打切向力绕行
        mag_m = calc_rep_mag(sonar_dm)
        if mag_m > 0:
            fx = -mag_m  # 斥力：向后推
            fy = mag_m * self.vortex_weight * target_dir # 切向力：目标在哪边就往哪边推
            F_rep_m = (fx, fy)
            F_rep_x += fx; F_rep_y += fy

        # 3.2 左侧超声波 (左侧遇阻)：优先往【传感器反向(右)】打切向力
        mag_l = calc_rep_mag(sonar_dl)
        if mag_l > 0:
            fx = mag_l * 0.2 # 稍微给点前向滑动推力防卡死
            fy = -mag_l * self.vortex_weight # 切向力：强制向右推
            F_rep_l = (fx, fy)
            F_rep_x += fx; F_rep_y += fy

        # 3.3 右侧超声波 (右侧遇阻)：优先往【传感器反向(左)】打切向力
        mag_r = calc_rep_mag(sonar_dr)
        if mag_r > 0:
            fx = mag_r * 0.2
            fy = mag_r * self.vortex_weight # 切向力：强制向左推
            F_rep_r = (fx, fy)
            F_rep_x += fx; F_rep_y += fy

        # --- 4. 向量合成与运动学输出 ---
        F_total_x = F_att_x + F_rep_x
        F_total_y = F_att_y + F_rep_y

        # [重要修改] 转向 w 必须改回使用合力 (F_total) 来计算角度！
        # 否则斥力算出来了也无法影响车头转向
        w = self.kp_w * math.atan2(F_total_y, F_total_x)
        
        v = self.max_v if F_total_x > 0 else 0.0

        # --- 5. 接口打包 ---
        force_info = {
            'F_att'   : (F_att_x,   F_att_y),
            'F_rep_l' : F_rep_l,
            'F_rep_m' : F_rep_m,
            'F_rep_r' : F_rep_r,
            'F_total' : (F_total_x, F_total_y), 
        }

        return v, w, target_x, target_y, force_info