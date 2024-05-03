import numpy as np
import cv2
from rtgym import DEFAULT_CONFIG_DICT
from rtgym import RealTimeGymInterface
from gymnasium.spaces import Box
from threading import Thread

# Dummy RC Drone for simulation purposes
class DummyRCDrone:
    """
    A dummy RC drone class to simulate the drone's state and control.
    """
    def __init__(self):
        self.pos = np.array([0.0, 0.0], dtype=np.float32)

    def send_control(self, vel_x, vel_y):
        """
        Simulates sending control to the drone. 
        In this case, it simply updates the drone's position.
        """
        self.pos += np.array([vel_x, vel_y]) * 0.05

    def get_observation(self):
        """
        Returns the current position of the drone.
        """
        return self.pos

# Custom RTGym interface
class DummyRCDroneInterface(RealTimeGymInterface):
    """
    Custom RTGym interface for the dummy RC drone.
    This class handles the interaction between the RTGym environment and the drone.
    """
    def __init__(self):
        self.rc_drone = None
        self.target = np.array([0.0, 0.0], dtype=np.float32)
        self.initialized = False
        self.blank_image = np.ones((500, 500, 3), dtype=np.uint8) * 255
        self.rendering_thread = Thread(target=self._rendering_thread, args=(), kwargs={}, daemon=True)

    def _rendering_thread(self):
        """
        A separate thread for rendering the environment.
        """
        from time import sleep
        while True:
            sleep(0.1)
            self.render()

    # ... [Include all the other methods from the previous code snippet here]

# Configuration for the RTGym interface
my_config = DEFAULT_CONFIG_DICT.copy()
my_config["interface"] = DummyRCDroneInterface
my_config["time_step_duration"] = 0.05
my_config["start_obs_capture"] = 0.05
my_config["time_step_timeout_factor"] = 1.0
my_config["ep_max_length"] = 100
my_config["act_buf_len"] = 4
my_config["reset_act_buf"] = False
my_config["benchmark"] = True
my_config["benchmark_polyak"] = 0.2

# Function to process key presses for manual control
def process_key_press():
    """
    Processes key presses and returns an action based on the input.
    'a' for left movement, 'd' for right movement.
    """
    key = cv2.waitKey(1)
    if key == ord('a'):  # Left key
        return np.array([-1.0, 0.0])
    elif key == ord('d'):  # Right key
        return np.array([1.0, 0.0])
    else:
        return np.array([0.0, 0.0])

# Main loop for manual control
def main():
    """
    The main function for running the manual control loop.
    It initializes the environment and processes user inputs for controlling the drone.
    """
    interface = DummyRCDroneInterface()
    env = interface.get_env(my_config)

    done = False
    while not done:
        action = process_key_press()
        obs, rew, done, info = env.step(action)
        env.render()

if __name__ == "__main__":
    main()
