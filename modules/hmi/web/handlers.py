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
import os

import flask
import flask_restful
import flask_socketio

import config
import hardware_api
import module_api
import ros_service_api
import runtime_status_api
import tool_api

app = flask.Flask(__name__)
app.secret_key = str(datetime.datetime.now())
socketio = flask_socketio.SocketIO(app)


@app.template_filter('log_file_shortcut')
def log_file_shortcut(filename):
    HEADING = 'data/log/'
    return filename[len(HEADING):] if filename.startswith(HEADING) else filename


# Web page handlers.
@app.route('/')
def index_page():
    """Handler of index page."""
    return flask.render_template('index.tpl', conf_pb=config.Config.get_pb())


@app.route('/module_card/<string:module_name>')
def module_card(module_name):
    """Handler of module card."""
    return flask.render_template(
        'cards/module_detail.tpl', module=config.Config.get_module(module_name))


# Restful API handlers.
api = flask_restful.Api(app)
api.add_resource(hardware_api.HardwareApi,
                 '/hardware_api/<string:hardware_name>')
api.add_resource(module_api.ModuleApi, '/module_api/<string:module_name>')
api.add_resource(ros_service_api.RosServiceApi,
                 '/ros_service_api/<string:cmd_name>')
api.add_resource(runtime_status_api.RuntimeStatusApi, '/runtime_status_api')
api.add_resource(tool_api.ToolApi, '/tool_api/<string:tool_name>')
