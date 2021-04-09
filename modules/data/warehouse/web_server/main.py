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
import os
import pickle
import sys

import flask
import gflags
import gunicorn.app.base

import add_pythonpath
import display_util
from modules.data.proto.task_pb2 import Task
from modules.data.proto.warehouse_query_pb2 import SearchRequest, SearchResponse
from mongo_util import Mongo
from query_task import WarehouseQuery


gflags.DEFINE_string('host', '0.0.0.0', 'Web host IP.')
gflags.DEFINE_integer('port', 8887, 'Web host port.')
gflags.DEFINE_integer('workers', 5, 'Web host workers.')
gflags.DEFINE_boolean('debug', False, 'Enable debug mode.')
gflags.DEFINE_integer('page_size', 20, 'Search results per page.')

app = flask.Flask(__name__)
app.secret_key = str(datetime.datetime.now())
app.jinja_env.filters.update(display_util.utils)


class Config(object):
    """Global config."""

    conf = {
        'vehicles': None,
        'topics': None,
    }

    @classmethod
    def build_conf_cache(cls):
        """Build config from DB and cache it in file"""
        CONF_FILE = './generated_data_warehouse.conf'
        if os.path.exists(CONF_FILE):
            with open(CONF_FILE, 'rb') as pickle_in:
                cls.conf = pickle.load(pickle_in)
        else:
            vehicles = set()
            topics = set()
            col = Mongo.collection(gflags.FLAGS.collection)
            for doc in col.find({}, {'info.vehicle.name': 1, 'topics': 1}):
                task = Mongo.doc_to_pb(doc, Task())
                topics.update(task.topics)
                if task.info.vehicle.name:
                    vehicles.add(task.info.vehicle.name)
            cls.conf = {
                'vehicles': sorted(vehicles),
                'topics': sorted(topics),
            }
            with open(CONF_FILE, 'wb') as pickle_out:
                pickle.dump(cls.conf, pickle_out)


@app.route('/')
@app.route('/search')
@app.route('/search/<int:page_idx>', methods=['GET', 'POST'])
def search_hdl(page_idx=1):
    """Handler of the data search page."""
    form = flask.request.form
    request_dict = {k:v for k, v in form.items() if v}
    # Convert checkboxes in form to a list.
    if 'topics' in form:
        request_dict['topics'] = form.getlist('topics')
    request = Mongo.doc_to_pb(request_dict, SearchRequest())
    request.fields.extend([
        'id',
        'start_time',
        'end_time',
        'loop_type',
        'info.vehicle.name',
        'info.environment.map_name',
        'mileage'
    ])
    request.count = gflags.FLAGS.page_size
    request.offset = gflags.FLAGS.page_size * (page_idx - 1)

    response = WarehouseQuery.search(request)
    return flask.render_template('search.tpl', conf=Config.conf,
                                 request=request, response=response)


@app.route('/task')
@app.route('/task/<path:task_id>')
def task_hdl(task_id):
    """Handler of the task detail page."""
    return flask.render_template('task.tpl',
                                 task=WarehouseQuery.get_task(task_id))


class FlaskApp(gunicorn.app.base.BaseApplication):
    """A wrapper to run flask app."""
    def __init__(self, flask_app):
        flask_app.debug = gflags.FLAGS.debug
        self.application = flask_app
        super(FlaskApp, self).__init__()

    def load_config(self):
        """Load config."""
        self.cfg.set('bind', '{}:{}'.format(gflags.FLAGS.host,
                                            gflags.FLAGS.port))
        self.cfg.set('workers', gflags.FLAGS.workers)
        self.cfg.set('proc_name', 'ApolloData')
        self.cfg.set('worker_class', 'gevent')

    def load(self):
        """Load app."""
        return self.application


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    Config.build_conf_cache()
    if gflags.FLAGS.debug:
        app.run(gflags.FLAGS.host, gflags.FLAGS.port, gflags.FLAGS.debug)
    else:
        FlaskApp(app).run()
