#!/usr/bin/env python3

# ****************************************************************************
# Copyright 2020 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************
# -*- coding: utf-8 -*-
import os
import wave
from cyber.python.cyber_py3 import cyber
from modules.drivers.microphone.proto.audio_pb2 import AudioData

RESPEAKER_CHANNEL = "/apollo/sensor/microphone"
WAV_SAVING_PATH = "/tmp"


def save_to_wave(frames, filepath, sample_width, sample_rate, n_channels=1):
    """Save frame to file.wave"""
    with wave.open(filepath, 'wb') as wf:
        wf.setnchannels(n_channels)
        wf.setsampwidth(sample_width)
        wf.setframerate(sample_rate)
        wf.writeframes(frames)


def callback(audio):
    sample_width = audio.microphone_config.sample_width
    sample_rate = audio.microphone_config.sample_rate
    print("=" * 40)
    print(audio.header)
    print("Saving wave files under " + WAV_SAVING_PATH + "...")
    for idx, channel_data in enumerate(audio.channel_data):
        file_path = os.path.join(WAV_SAVING_PATH, "seq_{}_channel_{}.wav".format(audio.header.sequence_num, idx))
        save_to_wave(channel_data.data, file_path, sample_width, sample_rate, 1)
    print("Done...")


def run():
    print("=" * 120)
    test_node = cyber.Node("audiosaver")
    test_node.create_reader(RESPEAKER_CHANNEL, AudioData, callback)
    test_node.spin()


if __name__ == '__main__':
    cyber.init()
    run()
    cyber.shutdown()
