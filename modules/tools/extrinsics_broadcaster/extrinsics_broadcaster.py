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
Broadcaster static transform
"""

import sys
import os
import yaml


def main():
    """Main function.

    Reading transform info from a yaml file and publish to tf2
    """
    if len(sys.argv) == 1:
        print "error: no extrinsics yaml file given"
        print "usage: python extrinsics_broadcaster.py extrinsic_example.yaml"
        return

    file_path = open(sys.argv[1])
    transform_stamped = yaml.safe_load(file_path)
    command = 'rosrun tf2_ros static_transform_publisher '\
        '%f %f %f %f %f %f %f %s %s' % (transform_stamped['transform']['translation']['x'],
                                        transform_stamped['transform']['translation']['y'],
                                        transform_stamped['transform']['translation']['z'],
                                        transform_stamped['transform']['rotation']['x'],
                                        transform_stamped['transform']['rotation']['y'],
                                        transform_stamped['transform']['rotation']['z'],
                                        transform_stamped['transform']['rotation']['w'],
                                        transform_stamped['header']['frame_id'],
                                        transform_stamped['child_frame_id'])

    print command
    ret = os.system(command)
    print ret


if __name__ == "__main__":
    main()
