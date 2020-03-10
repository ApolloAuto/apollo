#!/usr/bin/env python3

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

import os

import urllib.request
import urllib.parse
import urllib.error


URL_LIST = [
    "https://github.com/ApolloAuto/apollo/releases/download/v1.5.0/demo_1.5.bag",
    "https://github.com/ApolloAuto/apollo/releases/download/v2.0.0/demo_2.0.bag",
    "https://github.com/ApolloAuto/apollo/releases/download/v2.0.0/apollo_2.0_camera_sample.bag",
    "https://github.com/ApolloAuto/apollo/releases/download/v2.5.0/demo_2.5.bag",
    "https://github.com/ApolloAuto/apollo/releases/download/v1.0.0/demo_1.0.bag",
    "https://github.com/ApolloAuto/apollo/releases/download/v3.5.0/demo_3.5.record",
]

URL_DICT = {}
for link in URL_LIST:
    name = link.split("/")[-1]
    URL_DICT[name] = link


def retrieve_rosbag(bagname):
    if bagname not in URL_DICT:
        print(
            "bag[%s] is unknown, use one of the following rosbag names:\n%s" %
            (bagname, ", ".join([name for name in URL_DICT.keys()])))
        return False
    url = URL_DICT[bagname]
    print("Downloading from %s" % url)
    ret = os.system('wget %s -O %s' % (url, bagname))
    return ret == 0


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
        description='retrieve demo rosbag file from remote')
    parser.add_argument(
        'name',
        type=str,
        help='rosbag names. You can choose one of [%s]' % ", ".join(
            [name for name in URL_DICT.keys()]))
    args = parser.parse_args()
    if retrieve_rosbag(args.name):
        print("Download %s success" % args.name)
    else:
        print("Download %s failed" % args.name)
