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
"""HTTP request handlers."""

import datetime
import httplib

import flask
import flask_socketio

from config import Config
from runtime_status import RuntimeStatus
from socketio_api import SocketIOApi

app = flask.Flask(__name__)
app.secret_key = str(datetime.datetime.now())
socketio = flask_socketio.SocketIO(app, async_mode='gevent')


# Web page handlers.
@app.route('/')
def index_page():
    """Handler of index page."""
    protocol = 'https' if Config.get_pb().server.https.enabled else 'http'
    return flask.render_template('index.tpl', conf=Config, protocol=protocol)


@app.route('/runtime_status', methods=['GET', 'POST'])
def runtime_status():
    """
    Handler of runtime_status.
    Return current runtime status on GET request.
    Update runtime status on POST request which brings a RuntimeStatus json.
    """
    if flask.request.method == 'GET':
        return flask.jsonify(RuntimeStatus.status_json())
    else:
        RuntimeStatus.update(flask.request.get_json())
    return ('OK', httplib.OK)


# SocketIO handlers.
@socketio.on('socketio_api', namespace='/io_frontend')
def frontend_handler(socketio_request_json):
    """Handler of SocketIO request for frontend."""
    SocketIOApi.execute(socketio_request_json)
