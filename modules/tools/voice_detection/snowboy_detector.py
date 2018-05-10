#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
Detect voice and publish VoiceDetectionResponse.
"""

import sys

import rospy
import snowboydetect

import common.proto_utils as proto_utils
from modules.dreamview.proto import voice_detection_pb2


class SnowboyDetector(object):
    """
    Detect voice command with snowboy. For more information, please refer to
        https://github.com/Kitt-AI/snowboy
    """
    def __init__(self):
        CONF_FILE = '/apollo/modules/tools/voice_detection/voice_detection.conf'
        config = proto_utils.get_pb_from_text_file(
            CONF_FILE, voice_detection_pb2.VoiceDetectionConf())

        models = []
        self.hotwords = []
        for model in config.snowboy_models:
            models.append(model.path)
            self.hotwords.extend(model.hotwords)
        models_str = ','.join(models).encode()

        # TODO(xiaoxq): Currently we only support single detector for all users.
        self.detector = snowboydetect.SnowboyDetect(
            resource_filename=config.snowboy_resource.encode(),
            model_str=models_str)
        self.detector.SetAudioGain(config.audio_gain)

        sensitivity_str = '%.2f' % config.sensitivity
        self.detector.SetSensitivity(
            ','.join([sensitivity_str] * self.detector.NumHotwords()))

        self.voice_detection_pub = rospy.Publisher(
            '/apollo/hmi/voice_detection_response',
            voice_detection_pb2.VoiceDetectionResponse, queue_size=1)

    def voice_detection_request_callback(self, data):
        """New message received"""
        request = voice_detection_pb2.VoiceDetectionRequest()
        request.CopyFrom(data)
        hotword_index = self.detector.RunDetection(request.wav_stream)
        if hotword_index > 0:
            hotword = self.hotwords[hotword_index - 1]
            response = voice_detection_pb2.VoiceDetectionResponse()
            response.id = request.id
            response.action = hotword.action
            self.voice_detection_pub.publish(response)
            print 'Action {} is triggered!'.format(hotword.action)

def main():
    """Main rosnode"""
    rospy.init_node('voice_detector', anonymous=True)
    detector = SnowboyDetector()
    rospy.Subscriber('/apollo/hmi/voice_detection_request',
                     voice_detection_pb2.VoiceDetectionRequest,
                     detector.voice_detection_request_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
