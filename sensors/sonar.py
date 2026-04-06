from gpiozero import DistanceSensor

class SonarArray:
    def __init__(self, config):
        """Initialize the ultrasonic sensors based on the provided configuration"""
        print("[*] Initializing ultrasonic sensors...")
        self.left_sonar = DistanceSensor(
            echo=config['hardware']['sonar_left_echo'], 
            trigger=config['hardware']['sonar_left_trig'], 
            max_distance=2.0
        )
        self.right_sonar = DistanceSensor(
            echo=config['hardware']['sonar_right_echo'], 
            trigger=config['hardware']['sonar_right_trig'], 
            max_distance=2.0
        )

    def get_distances(self):
        """Return the distances from the left and right ultrasonic sensors (unit: meters)"""
        return self.left_sonar.distance, self.right_sonar.distance