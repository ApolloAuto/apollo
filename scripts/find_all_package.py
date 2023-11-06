# !/usr/bin/env python3
###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
external py script for shell
"""
import os
from pathlib import Path

TARGET_PATH=[]
ROOT="/apollo"

def search_cyberfile(root):
    """
    search all cyberfile/package in workspace
    """
    global TARGET_PATH 
    files = os.listdir(root)
    has_cyberfile = False

    if "cyberfile.xml" in files:
            TARGET_PATH.append(root)
            has_cyberfile = True

    for f in files:
        if f.startswith("."):
            continue
        f_desc = Path(os.path.join(root, f))
        if f_desc.is_dir() and not f_desc.is_symlink():
            search_cyberfile(str(f_desc))

if __name__ == "__main__":
    search_cyberfile(ROOT)
    print(" ".join([i.replace("{}/".format(ROOT), "") for i in TARGET_PATH]))