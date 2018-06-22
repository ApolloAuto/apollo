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
"""Serve data imported in MongoDB."""

import datetime
import sys

import flask
import gflags

import add_pythonpath
import display_util
from stat_task import TaskCalculator


gflags.DEFINE_integer('port', 8887, 'Web host port.')
gflags.DEFINE_string('task_dir', None, 'Task directory.')

app = flask.Flask(__name__)
app.secret_key = str(datetime.datetime.now())
app.jinja_env.filters.update(display_util.utils)
task = None

@app.route('/')
def task_hdl():
    """Handler of the task detail page."""
    return flask.render_template('task.tpl', task=task)

if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    task = TaskCalculator.stat(gflags.FLAGS.task_dir)
    app.run(host='0.0.0.0', port=gflags.FLAGS.port)
