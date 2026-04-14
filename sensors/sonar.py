from gpiozero import DistanceSensor
from collections import deque


class MovingAverageFilter:
    """Simple moving average filter backed by a fixed-size circular buffer."""

    def __init__(self, window_size: int = 5):
        self._window_size = window_size
        self._buf = deque(maxlen=window_size)

    def update(self, value: float) -> float:
        """Push a new raw reading and return the current filtered average."""
        self._buf.append(value)
        return sum(self._buf) / len(self._buf)

    def reset(self):
        self._buf.clear()


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

        # --- Moving average filters (one per sensor) ---
        self._left_filter  = MovingAverageFilter(window)
        self._mid_filter   = MovingAverageFilter(window)
        self._right_filter = MovingAverageFilter(window)

        print(f"[*] Moving-average filter window: {window} samples")

    def get_distances(self):
        """Return the filtered distances from the left, middle, and right
        ultrasonic sensors (unit: metres).

        Returns:
            tuple[float, float, float]: (left, mid, right) filtered distances.
        """
        left  = self._left_filter.update(self.left_sonar.distance)
        mid   = self._mid_filter.update(self.mid_sonar.distance)
        right = self._right_filter.update(self.right_sonar.distance)
        return left, mid, right
