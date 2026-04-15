from gpiozero import DistanceSensor
from collections import deque


class SonarArray:
    def __init__(self, config):
        """Initialize the ultrasonic sensors and moving-average filters
        based on the provided configuration."""
        print("[*] Initializing ultrasonic sensors...")

        hw = config['hardware']
        window = hw.get('sonar_moving_avg_window', 5)

        # --- Sensors ---
        self.left_sonar = DistanceSensor(
            echo=hw['sonar_left_echo'],
            trigger=hw['sonar_left_trig'],
            max_distance=2.0
        )
        self.mid_sonar = DistanceSensor(
            echo=hw['sonar_mid_echo'],
            trigger=hw['sonar_mid_trig'],
            max_distance=2.0
        )
        self.right_sonar = DistanceSensor(
            echo=hw['sonar_right_echo'],
            trigger=hw['sonar_right_trig'],
            max_distance=2.0
        )

    def get_distances(self):
        return self.left_sonar.distance, self.mid_sonar.distance, self.right_sonar.distance
