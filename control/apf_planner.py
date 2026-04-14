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

        # If the target angle exceeds this threshold the robot stops driving
        # forward and spins in place until it's roughly facing the target.
        spin_deg = config['apf'].get('spin_threshold_deg', 40)
        self.spin_threshold = math.radians(spin_deg)

    def compute_command(self, uwb_dl, uwb_dr, sonar_dl, sonar_dm, sonar_dr):
        """Input sensor data and output the desired linear and angular velocities"""
       
        # --- 1. Calculate local repulsion and vortex forces ---
        F_local_x, F_local_y = 0.0, 0.0
       
        if sonar_dl < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dl - 1.0/self.safe_radius) / (sonar_dl**2)
            F_local_y -= rep_mag
            F_local_x += rep_mag * self.vortex_weight

        # Middle sensor: pure rearward (−x) repulsion with no lateral vortex component
        if sonar_dm < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dm - 1.0/self.safe_radius) / (sonar_dm**2)
            F_local_x -= rep_mag

        if sonar_dr < self.safe_radius:
            rep_mag = self.k_rep * (1.0/sonar_dr - 1.0/self.safe_radius) / (sonar_dr**2)
            F_local_y += rep_mag
            F_local_x += rep_mag * self.vortex_weight

        # --- 2. Local target position calculation ---
        # Calculate the target position in the robot's local frame based on UWB distances
        target_y = (uwb_dr**2 - uwb_dl**2) / (2 * self.L)
        val_for_sqrt = uwb_dl**2 - (target_y - self.L/2)**2
        target_x = math.sqrt(max(val_for_sqrt, 0.001))

        dist_target = math.hypot(target_x, target_y)
       
        if dist_target > 0.2:
            F_att_x = self.k_att * (target_x / dist_target)
            F_att_y = self.k_att * (target_y / dist_target)
        else:
            F_att_x, F_att_y = 0.0, 0.0

        # --- 3. Total force calculation ---
        F_total_x = F_att_x + F_local_x
        F_total_y = F_att_y + F_local_y

        if dist_target > 0.2:
            desired_w = math.atan2(F_total_y, F_total_x)
            w = self.kp_w * desired_w

            # If the target is far to the side, spin in place rather than arc.
            # Use the raw target angle (not the force angle) so obstacle forces
            # don't accidentally suppress spinning when the person moves sideways.
            target_angle = abs(math.atan2(target_y, target_x))
            if target_angle > self.spin_threshold:
                v = 0.0   # spin in place until roughly facing the target
            else:
                v = self.max_v if F_total_x > 0 else 0.0
        else:
            v, w = 0.0, 0.0

        return v, w, target_x, target_y
