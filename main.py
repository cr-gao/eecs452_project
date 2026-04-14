import time
import yaml
import sys
from sensors.sonar import SonarArray
from sensors.uwb import DualUWBManager
from control.apf_planner import APFPlanner
from hardware.motor_driver import MotorController

def load_config(path="config.yaml"):
    try:
        with open(path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"[!] Failed to load config file: {e}")
        sys.exit(1)

def main():
    print("="*45)
    print("Launching APF-Based UWB Follow Robot System")
    print("="*45)

    # 1. Load configuration
    config = load_config()

    STOP_THRESHOLD = 0.5

    # 2. Initialize hardware interfaces and control modules
    sonar = SonarArray(config)
    uwb = DualUWBManager(config)
    planner = APFPlanner(config)
    chassis = MotorController()

    print("\n[*] Entering main control loop... (Press Ctrl+C to exit)\n")
    
    # 3. Main control loop (target frequency ~10Hz)
    try:
        while True:
            loop_start = time.time()

            # --- A. Read sensor data ---
            sonar_dl, sonar_dm, sonar_dr = sonar.get_distances()
            uwb_dl, uwb_dr = uwb.get_distances()

            # Emergency stop check
            # Find the minimum distance from all sensors
            min_dist = min(sonar_dl, sonar_dm, sonar_dr, uwb_dl, uwb_dr)

            if min_dist <= STOP_THRESHOLD:
                print(f"[Emergency stop triggered, min distance] {min_dist:.2f}m <= {STOP_THRESHOLD}m, stopping robot!")

                # Send zero velocity command to stop the robot immediately
                chassis.send_cmd_vel(0, 0)

                # Sleep for a short duration to prevent spamming the stop command and allow the robot to come to a halt
                elapsed = time.time() - loop_start
                sleep_time = max(0.1 - elapsed, 0)
                time.sleep(sleep_time)
                continue

            # --- B. Compute motion commands ---
            v, w, tgt_x, tgt_y = planner.compute_command(uwb_dl, uwb_dr, sonar_dl, sonar_dm, sonar_dr)

            # --- C. Execute motion ---
            chassis.send_cmd_vel(v, w)

            # --- D. Terminal monitoring ---
            print(f"[Monitoring] UWB_L:{uwb_dl:.2f}m R:{uwb_dr:.2f}m | Target:(x:{tgt_x:.2f}, y:{tgt_y:.2f}) | Obstacle_L:{sonar_dl:.2f}m M:{sonar_dm:.2f}m R:{sonar_dr:.2f}m")

            # --- E. Loop frequency control ---
            elapsed = time.time() - loop_start
            sleep_time = max(0.1 - elapsed, 0)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n[*] Interrupt signal received, shutting down system...")
        # Add any necessary cleanup code here (e.g., stop motors, close serial ports)
        chassis.send_cmd_vel(0, 0)
        print("[*] System has been shut down safely.")

if __name__ == "__main__":
    main()