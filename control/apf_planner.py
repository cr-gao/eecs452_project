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

        # --- 1. Repulsion + vortex forces ---
        F_rep_l = (0.0, 0.0)
        F_rep_m = (0.0, 0.0)
        F_rep_r = (0.0, 0.0)
        F_local_x, F_local_y = 0.0, 0.0

        if sonar_dl < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dl - 1.0/self.safe_radius) / (sonar_dl**2)
            fx =  rep_mag * self.vortex_weight
            fy = -rep_mag
            F_rep_l = (fx, fy)
            F_local_x += fx; F_local_y += fy

        if sonar_dm < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dm - 1.0/self.safe_radius) / (sonar_dm**2)
            # 增加一个侧向的涡旋力，迫使小车向某一侧转弯 (比如向右)
            F_rep_m = (-rep_mag, rep_mag * self.vortex_weight) 
            F_local_x -= rep_mag; F_local_y += rep_mag * self.vortex_weight  # 打破平衡！

        if sonar_dr < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dr - 1.0/self.safe_radius) / (sonar_dr**2)
            fx =  rep_mag * self.vortex_weight
            fy =  rep_mag
            F_rep_r = (fx, fy)
            F_local_x += fx; F_local_y += fy

        # --- 2. Target position (robot-local frame) from UWB geometry ---
        # 在计算 target_y 之前加入限幅
        diff = uwb_dr - uwb_dl
        diff = max(min(diff, self.L * 0.99), -self.L * 0.99) # 留一点余量防止除零或负数
        uwb_dr = uwb_dl + diff
        target_y = (uwb_dr**2 - uwb_dl**2) / (2 * self.L)
        val_for_sqrt = uwb_dl**2 - (target_y - self.L/2)**2
        target_x = math.sqrt(max(val_for_sqrt, 0.001))
        dist_target = math.hypot(target_x, target_y)

        if dist_target > 0.2:
            F_att_x = self.k_att * (target_x / dist_target)
            F_att_y = self.k_att * (target_y / dist_target)
        else:
            F_att_x, F_att_y = 0.0, 0.0

        # --- 3. Total force ---
        F_total_x = F_att_x + F_local_x
        F_total_y = F_att_y + F_local_y

        # --- 4. Velocity commands ---
        if dist_target > 0.2:
            w = self.kp_w * math.atan2(F_total_y, F_total_x)
            v = self.max_v if F_total_x > 0 else 0.0
        else:
            v, w = 0.0, 0.0

        force_info = {
            'F_att'   : (F_att_x,   F_att_y),
            'F_rep_l' : F_rep_l,
            'F_rep_m' : F_rep_m,
            'F_rep_r' : F_rep_r,
            'F_total' : (F_total_x, F_total_y),
        }

        return v, w, target_x, target_y, force_info