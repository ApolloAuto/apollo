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
"""Entry point of the server."""

import google.apputils.app

from config import Config
from ros_bridge_api import RosBridgeApi
from runtime_status import RuntimeStatus
import handlers


def main(argv):
    """App entry point."""
    conf = Config.get_pb()
    # Module initialization.
    RuntimeStatus.reset(True)
    RosBridgeApi.init_ros()
    # Start web server.
    return handlers.socketio.run(handlers.app,
                                 host=conf.server.host, port=conf.server.port)


if __name__ == '__main__':
    google.apputils.app.run()
