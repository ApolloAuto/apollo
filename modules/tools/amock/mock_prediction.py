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

import keyboard

from cyber.python.cyber_py3 import cyber
from cyber.python.cyber_py3 import cyber_time, cyber_timer
from modules.common_msgs.prediction_msgs import prediction_obstacle_pb2

TOPIC = "/apollo/prediction"
PERIOD = 100   # ms

def publish(writer, sequence_num):
    try:
        prediction_obstacles = prediction_obstacle_pb2.PredictionObstacles()

        prediction_obstacles.header.timestamp_sec = cyber_time.Time.now().to_sec()
        prediction_obstacles.header.module_name = 'mock_prediction'
        prediction_obstacles.header.sequence_num = sequence_num

        writer.write(prediction_obstacles)
    except Exception as e:
        print(f"[Error] Failed to publish message: {e}")


def main():
    try:
        cyber.init()
        node = cyber.Node("mock_prediction")
        writer = node.create_writer(
            TOPIC, prediction_obstacle_pb2.PredictionObstacles)

        sequence_num = 0

        def callback():
            """publish message"""
            nonlocal sequence_num
            publish(writer, sequence_num)
            sequence_num += 1

        ct = cyber_timer.Timer(PERIOD, callback, 0)
        ct.start()

        print("Press 'ESC' or 'Q' to exit...")

        keyboard.wait(lambda e: e.name == 'esc' or e.name == 'q')

        print("Exiting...")

    except Exception as e:
        print(f"[Error] {e}")

    finally:
        ct.stop()
        cyber.shutdown()


if __name__ == '__main__':
    main()
