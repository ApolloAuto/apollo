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

import collections
import datetime
import os
import pickle
import sys

import flask
import gflags
import gunicorn.app.base
import pymongo

import add_pythonpath
import display_util
import records_util
from modules.data.proto.record_pb2 import Record
from mongo_util import Mongo


gflags.DEFINE_string('host', '0.0.0.0', 'Web host IP.')
gflags.DEFINE_integer('port', 8000, 'Web host port.')
gflags.DEFINE_integer('workers', 5, 'Web host workers.')
gflags.DEFINE_boolean('debug', False, 'Enable debug mode.')
gflags.DEFINE_integer('page_size', 20, 'Search results per page.')

gflags.DEFINE_string('mongo_collection_name', 'records',
                     'MongoDB collection name.')

app = flask.Flask(__name__)
app.secret_key = str(datetime.datetime.now())
app.jinja_env.filters.update(display_util.utils)


@app.route('/')
@app.route('/tasks/<int:page_idx>')
def tasks_hdl(page_idx=1):
    """Handler of the task list page."""
    G = gflags.FLAGS
    mongo_col = Mongo.collection(G.mongo_collection_name)
    task_dirs = {doc['dir'] for doc in mongo_col.find({}, {'dir': 1})}
    page_count = (len(task_dirs) + G.page_size - 1) / G.page_size
    if page_idx > page_count:
        flask.flash('Page index out of bound')
        return flask.render_template('base.tpl')

    offset = G.page_size * (page_idx - 1)
    task_dirs = sorted(list(task_dirs), reverse=True)
    query = {'dir': {'$in': task_dirs[offset : offset + G.page_size]}}
    kFields = {
        'dir': 1,
        'header.begin_time': 1,
        'header.end_time': 1,
        'header.size': 1,
        'hmi_status.current_mode': 1,
        'hmi_status.current_map': 1,
        'hmi_status.current_vehicle': 1,
        'disengagements': 1,
        'drive_events': 1,
        'stat.mileages': 1,
    }
    task_records = collections.defaultdict(list)
    for doc in mongo_col.find(query, kFields):
        task_records[doc['dir']].append(Mongo.doc_to_pb(doc, Record()))
    tasks = [records_util.CombineRecords(records)
             for records in task_records.values()]
    tasks.sort(key=lambda task: task.dir, reverse=True)
    return flask.render_template('records.tpl', page_count=page_count,
                                 current_page=page_idx, records=tasks,
                                 is_tasks=True)

@app.route('/task/<path:task_path>')
def task_hdl(task_path):
    """Handler of the task detail page."""
    mongo_col = Mongo.collection(gflags.FLAGS.mongo_collection_name)
    docs = mongo_col.find({'dir': os.path.join('/', task_path)})
    records = [Mongo.doc_to_pb(doc, Record()) for doc in docs]
    task = records_util.CombineRecords(records)
    return flask.render_template('record.tpl', record=task, sub_records=records)

@app.route('/records')
@app.route('/records/<int:page_idx>')
def records_hdl(page_idx=1):
    """Handler of the record list page."""
    G = gflags.FLAGS
    kFields = {
        'path': 1,
        'header.begin_time': 1,
        'header.end_time': 1,
        'header.size': 1,
        'hmi_status.current_mode': 1,
        'hmi_status.current_map': 1,
        'hmi_status.current_vehicle': 1,
        'disengagements': 1,
        'drive_events': 1,
        'stat.mileages': 1,
    }
    kSort = [('header.begin_time', pymongo.DESCENDING)]

    docs = Mongo.collection(G.mongo_collection_name).find({}, kFields)
    page_count = (docs.count() + G.page_size - 1) / G.page_size
    offset = G.page_size * (page_idx - 1)
    records = [Mongo.doc_to_pb(doc, Record())
               for doc in docs.sort(kSort).skip(offset).limit(G.page_size)]
    return flask.render_template('records.tpl', page_count=page_count,
                                 current_page=page_idx, records=records)


@app.route('/record/<path:record_path>')
def record_hdl(record_path):
    """Handler of the record detail page."""
    mongo_col = Mongo.collection(gflags.FLAGS.mongo_collection_name)
    doc = mongo_col.find_one({'path': os.path.join('/', record_path)})
    record = Mongo.doc_to_pb(doc, Record())
    return flask.render_template('record.tpl', record=record)


class FlaskApp(gunicorn.app.base.BaseApplication):
    """A wrapper to run flask app."""
    def __init__(self, flask_app):
        flask_app.debug = gflags.FLAGS.debug
        self.application = flask_app
        super(FlaskApp, self).__init__()

    def load_config(self):
        """Load config."""
        G = gflags.FLAGS
        self.cfg.set('bind', '{}:{}'.format(G.host, G.port))
        self.cfg.set('workers', G.workers)
        self.cfg.set('proc_name', 'ApolloData')

    def load(self):
        """Load app."""
        return self.application


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    if gflags.FLAGS.debug:
        app.run(gflags.FLAGS.host, gflags.FLAGS.port, gflags.FLAGS.debug)
    else:
        FlaskApp(app).run()
