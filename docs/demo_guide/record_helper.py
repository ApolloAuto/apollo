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
"""
A script for downloading Apollo record files
"""

import os
import sys
import argparse
import subprocess

DOWNLOAD_LINK_PREFIX = "https://github.com/ApolloAuto/apollo/releases/download"

URL_LISTING = [
    "v1.0.0/demo_1.0.bag",
    "v1.5.0/demo_1.5.bag",
    "v2.0.0/demo_2.0.bag",
    "v2.0.0/apollo_2.0_camera_sample.bag",
    "v2.5.0/demo_2.5.bag",
    "v3.5.0/demo_3.5.record",
]


def build_urls():
    urls = {}
    for u in URL_LISTING:
        urls[os.path.basename(u)] = "{}/{}".format(DOWNLOAD_LINK_PREFIX, u)
    return urls


def download_record(record_name, urls):
    """
    Match and download record from urls, and save it locally
    """
    if record_name not in urls:
        print(
            "Unknown record: {}. Type \"{} --help\" on available records.".format(
                record_name,
                sys.argv[0]))
        return False
    url = urls[record_name]
    print("Downloading {}".format(url))
    result = subprocess.run(["wget", url, "-O", record_name])
    return result.returncode == 0


if __name__ == "__main__":
    urls = build_urls()
    name_desc = "record name. Available records: {}".format(
        ", ".join(u for u in urls))
    parser = argparse.ArgumentParser(
        description="A script for downloading Apollo demo records")
    parser.add_argument(
        "name",
        type=str,
        help=name_desc)
    args = parser.parse_args()

    requested_record = args.name
    success = download_record(requested_record, urls)
    if success:
        print("Successfully downloaded {}".format(requested_record))
    else:
        print("Bad luck, failed to download {}".format(requested_record))
