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
clear the extra -dev due to install rule
"""
import os
from pathlib import Path

output_dir = "/opt/apollo/neo/packages"

if __name__ == "__main__":
    for d in os.listdir(output_dir):
        d_wrapper = Path(os.path.join(output_dir, d))
        cyberfile = d_wrapper / "local" / "cyberfile.xml"
        if cyberfile.exists():
            content = None
            with cyberfile.open("r") as f:
                content = f.read()
            while "-dev-dev" in content:
                content = content.replace("-dev-dev", "-dev")
            with cyberfile.open("w+") as f:
                f.write(content)
