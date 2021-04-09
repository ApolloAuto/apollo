#!/usr/bin/env python
# -*- coding: UTF-8-*-
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
"""Import Apollo data into a MongoDB."""
import os
import sys

import gflags

import add_pythonpath
from mongo_util import Mongo
from stat_task import TaskCalculator


gflags.DEFINE_string('collection', 'apollo', 'MongoDB collection name.')

gflags.DEFINE_string('import_mode', 'upsert',
                     'Strategy when importing task to MongoDB:'
                     'insert: Insert only when it does not exist; '
                     'update: Update only when it exist; '
                     'upsert: Replace or insert.')


def import_task(task_dir, col=None):
    """
    Import a task.

    See https://api.mongodb.com/python/current/api/pymongo/collection.html for
    MongoDB collection document.
    As we used some advanced features, pymongo >= 3.0 is required.
    """
    mode = gflags.FLAGS.import_mode
    if mode == 'insert':
        # Check if the task has been imported.
        if col is None:
            col = Mongo.collection(gflags.FLAGS.collection)
        path_array = os.path.abspath(task_dir).split('/')
        if len(path_array) >= 2:
            guess_id = '{}/{}'.format(path_array[-2], path_array[-1])
            if col.find_one({'id': guess_id}, {'_id': 1}) is not None:
                sys.stderr.write('Task {} has already existed.\n'.format(
                    task_dir))
                return

    task = TaskCalculator.stat(task_dir)
    if task is None:
        sys.exit(-1)

    doc = Mongo.pb_to_doc(task)
    query = {'id': doc['id']}
    if col is None:
        col = Mongo.collection(gflags.FLAGS.collection)

    # TODO(xiaoxq): We use reprecated insert() and update() functions because
    # they support 'check_keys' parameter which doesn't reject '.' in keys.
    if mode == 'insert':
        col.insert(doc, check_keys=False)
    elif mode == 'update':
        col.update(query, doc, upsert=False, check_keys=False)
    elif mode == 'upsert':
        col.update(query, doc, upsert=True, check_keys=False)


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    task_dir = sys.argv[-1]
    import_task(task_dir)
