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
import ssl
import sys

import gflags

from config import Config
from ros_bridge_api import RosBridgeApi
from runtime_status import RuntimeStatus
import handlers


def main():
    """App entry point."""
    conf = Config.get_pb()
    # Module initialization.
    RuntimeStatus.reset(True)
    RosBridgeApi.init_ros()
    # Start web server.
    kwargs = {}
    https = conf.server.https
    if https.enabled:
        # See https://docs.python.org/2/library/ssl.html#ssl.wrap_socket
        kwargs = {
            'server_side': True,
            'ssl_version': ssl.PROTOCOL_TLSv1,
            'keyfile': Config.get_realpath(https.server_key),
            'certfile': Config.get_realpath(https.server_cert)
        }
        if https.client_cert_required:
            kwargs['cert_reqs'] = ssl.CERT_REQUIRED
    return handlers.socketio.run(handlers.app,
                                 host=conf.server.binding_ip,
                                 port=conf.server.port,
                                 **kwargs)


if __name__ == '__main__':
    # Parse gflags before main function.
    gflags.FLAGS(sys.argv)
    main()
