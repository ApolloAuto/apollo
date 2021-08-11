#!/usr/bin/env python3

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
Broadcaster static transform
"""

from subprocess import call
import sys

import yaml


def main():
    """Main function.

    Reading transform info from a yaml file and publish to tf2
    """
    if len(sys.argv) < 2:
        print('Usage: %s extrinsic_example.yaml' % sys.argv[0])
        return

    with open(sys.argv[1]) as fp:
        transform_stamped = yaml.safe_load(file_path)
        command = 'rosrun tf2_ros static_transform_publisher ' \
                  '%f %f %f %f %f %f %f %s %s' % \
                  (transform_stamped['transform']['translation']['x'],
                   transform_stamped['transform']['translation']['y'],
                   transform_stamped['transform']['translation']['z'],
                   transform_stamped['transform']['rotation']['x'],
                   transform_stamped['transform']['rotation']['y'],
                   transform_stamped['transform']['rotation']['z'],
                   transform_stamped['transform']['rotation']['w'],
                   transform_stamped['header']['frame_id'],
                   transform_stamped['child_frame_id'])

    print(command)

    try:
        return call(command, shell=True)
    except OSError as e:
        print(e)


if __name__ == '__main__':
    main()
