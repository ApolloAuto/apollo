#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""
Play car sounds
"""

import sys
import rospy
from std_msgs.msg import String
from modules.common.proto import car_sound_pb2
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import time

class CarSound(object):
    """
    car sound class
    """

    def __init__(self):
        self.entity = car_sound_pb2.SoundRequest()

        self.soundhandle = SoundClient()
        self.voice = 'voice_kal_diphone'
        self.lasttime = rospy.get_time()

    def callback_sound(self, data):
        """
        new sound request
        """
        print "New Sound Msg"
        self.entity.ParseFromString(data.data)
        print self.entity
        if self.entity.mode == car_sound_pb2.SoundRequest.SAY:
            self.soundhandle.say(self.entity.words, self.voice)
        elif self.entity.mode == car_sound_pb2.SoundRequest.BEEP:
            self.soundhandle.play(SoundRequest.NEEDS_PLUGGING)
        self.lasttime = rospy.get_time()

def main():
    """
    Main rosnode
    """

    rospy.init_node('car_sound', anonymous=True)

    sound = CarSound()

    time.sleep(1)
    sound.soundhandle.play(SoundRequest.NEEDS_UNPLUGGING)
    rospy.Subscriber('/apollo/carsound', String, sound.callback_sound)
    rospy.spin()


if __name__ == '__main__':
    main()
