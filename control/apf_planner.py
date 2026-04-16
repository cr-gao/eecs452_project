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

        # 转向：直接朝着目标点的物理坐标转，不再通过受力去绕圈子
        w = self.kp_w * math.atan2(target_y, target_x)
        
        # 线速度：只要算出来的 x 在车头前方，就直接给满速 (反正不管远近逻辑)
        v = self.max_v if target_x > 0 else 0.0

        # --- 3. 接口打包 (忽略所有斥力) ---
        force_info = {
            'F_att'   : (F_att_x,   F_att_y),
            'F_rep_l' : (0.0, 0.0),
            'F_rep_m' : (0.0, 0.0),
            'F_rep_r' : (0.0, 0.0),
            'F_total' : (F_att_x, F_att_y), # 没有斥力，总力就是引力
        }

        return v, w, target_x, target_y, force_info