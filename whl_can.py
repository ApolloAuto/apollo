#!/usr/bin/env python

# Copyright 2025 daohu527 <daohu527@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import curses
import threading
import time
import logging

from cyber.python.cyber_py3 import cyber
from modules.common_msgs.control_msgs import control_cmd_pb2

CONTROL_TOPIC = "/apollo/control"

SPEED_DELTA = 0.1
STEERING_RATE_DELTA = 1


class KeyboardController:
    """
    Curses-based keyboard controller, no root privileges required, suitable for Linux terminal environments.
    Listens for key presses in non-blocking mode:
      - w: Forward (accelerate)
      - s: Backward (decelerate)
      - a: Turn left (increase steering angle)
      - d: Turn right (decrease steering angle)
      - q: Exit the program
    """

    def __init__(self, screen, speed_delta=SPEED_DELTA, steering_rate_delta=STEERING_RATE_DELTA):
        self.screen = screen
        self.running = True
        self.control_cmd_msg = control_cmd_pb2.ControlCommand()
        self.speed = 0
        self.steering_rate = 0
        self.speed_delta = speed_delta
        self.steering_rate_delta = steering_rate_delta
        self.lock = threading.Lock()
        self.logger = logging.getLogger(__name__)

        # Key mapping: map keys using ASCII codes
        self.control_map = {
            ord('w'): self.move_forward,
            ord('s'): self.move_backward,
            ord('a'): self.turn_left,
            ord('d'): self.turn_right,
        }

    def get_control_cmd(self):
        """Returns the latest control command message."""
        with self.lock:
            return self.control_cmd_msg

    def start(self):
        """Starts keyboard listening, sets curses to non-blocking mode and starts the listening thread."""
        self.screen.nodelay(True)  # Set non-blocking input
        self.screen.keypad(True)
        self.screen.addstr(
            0, 0, "Keyboard control started, press 'q' to exit.    ")
        self.thread = threading.Thread(
            target=self._listen_keyboard, daemon=True)
        self.thread.start()

    def stop(self):
        """Stops keyboard listening."""
        with self.lock:
            self.running = False
        self.screen.addstr(
            1, 0, "Keyboard control stopped.                    ")

    def _listen_keyboard(self):
        """Loop reads keyboard input and calls the corresponding control method based on the key pressed."""
        while self.running:
            try:
                key = self.screen.getch()  # Non-blocking call
            except Exception as e:
                print(f"Error reading keyboard input: {e}")
                key = -1

            if key != -1:
                if key == ord('q'):
                    self.stop()
                    break
                elif key in self.control_map:
                    with self.lock:
                        self.control_map[key]()
                self.fill_control_cmd()
            time.sleep(0.05)

    def fill_control_cmd(self):
        """Updates the current speed and steering_rate to the protobuf message."""
        with self.lock:
            self.control_cmd_msg.speed = self.speed
            self.control_cmd_msg.steering_rate = self.steering_rate

    def move_forward(self):
        """Forward control: increase speed."""
        self.speed += self.speed_delta
        self.screen.addstr(2, 0, f"speed: {self.speed:.2f}    ")

    def move_backward(self):
        """Backward control: decrease speed."""
        self.speed -= self.speed_delta
        self.screen.addstr(2, 0, f"speed: {self.speed:.2f}    ")

    def turn_left(self):
        """Turn left control: increase steering angle."""
        self.steering_rate += self.steering_rate_delta
        self.screen.addstr(3, 0, f"steer: {self.steering_rate:.2f}    ")

    def turn_right(self):
        """Turn right control: decrease steering angle."""
        self.steering_rate -= self.steering_rate_delta
        self.screen.addstr(3, 0, f"steer: {self.steering_rate:.2f}    ")


def main(screen):
    # Configure logging at the program entry point
    logging.basicConfig(level=logging.INFO)
    cyber.init()
    node = cyber.Node("can_easy")
    writer = node.create_writer(CONTROL_TOPIC, control_cmd_pb2.ControlCommand)

    # Pre-print the fixed format lines
    screen.addstr(2, 0, "speed: 0.00    ")
    screen.addstr(3, 0, "steer: 0.00    ")

    controller = KeyboardController(screen)
    controller.start()

    try:
        while controller.running:
            cmd = controller.get_control_cmd()
            writer.write(cmd)
            time.sleep(0.1)
    except KeyboardInterrupt:
        controller.stop()
    finally:
        controller.stop()
        cyber.shutdown()

    screen.addstr(6, 0, "Program exited.                        ")


if __name__ == "__main__":
    # Use curses.wrapper to ensure proper initialization and cleanup of the curses environment
    curses.wrapper(main)
