import time
import yaml
import sys
from sensors.sonar import SonarArray
from sensors.uwb import DualUWBManager
from control.apf_planner import APFPlanner
# Import the virtual chassis for simulation instead of the real motor controller
from hardware.virtual_chassis import VirtualChassis

def load_config(path="config.yaml"):
    try:
        with open(path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"[!] Failed to load config file: {e}")
        sys.exit(1)

def main():
    print("="*45)
    print("Starting APF-Based UWB Follow Robot Simulation")
    print("="*45)

    config = load_config()

    sonar = SonarArray(config)
    uwb = DualUWBManager(config)
    planner = APFPlanner(config)
    
    # Initialize the virtual chassis for simulation instead of the real motor controller
    chassis = VirtualChassis()

    print("\n[*] Entering main control loop... (Please observe the robot in the popped-up window)\n")
    
    try:
        while True:
            loop_start = time.time()

            sonar_dl, sonar_dr = sonar.get_distances()
            uwb_dl, uwb_dr = uwb.get_distances()

            # Compute the motion command and also get the target coordinates for visualization
            v, w, tgt_x, tgt_y = planner.compute_command(uwb_dl, uwb_dr, sonar_dl, sonar_dr)

            # ▼ Change: pass the target coordinates (tgt_x, tgt_y) to the chassis for visualization ▼
            chassis.send_cmd_vel(v, w, tgt_x, tgt_y)

            # Terminal can still print information for comparison
            print(f"UWB_L:{uwb_dl:.2f} R:{uwb_dr:.2f} | Obstacle L:{sonar_dl:.2f} R:{sonar_dr:.2f}", end='\r')

            elapsed = time.time() - loop_start
            sleep_time = max(0.1 - elapsed, 0)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[*] Received interrupt signal, shutting down simulation...")

if __name__ == "__main__":
    main()