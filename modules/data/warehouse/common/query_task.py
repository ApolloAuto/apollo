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
"""Query task from a MongoDB."""
import os
import sys

import flask
import gflags
import pymongo

from modules.data.proto.task_pb2 import Task
from modules.data.proto.warehouse_query_pb2 import SearchRequest, SearchResponse
from mongo_util import Mongo


gflags.DEFINE_string('collection', 'tasks', 'MongoDB collection name.')


class WarehouseQuery(object):
    """Implementation of warehouse queries."""

    @staticmethod
    def search(request):
        """Search and return SearchResponse."""
        SEARCH_SORT = [('start_time', pymongo.DESCENDING),]

        fields = {field : 1 for field in request.fields}
        query = {}
        if request.vehicle_name:
            fields['info.vehicle.name'] = 1
            query['info.vehicle.name'] = request.vehicle_name.encode('utf-8')
        if request.map_name:
            fields['info.environment.map_name'] = 1
            query['info.environment.map_name'] = request.map_name.encode(
                'utf-8')
        if request.HasField('loop_type'):
            fields['loop_type'] = 1
            query['loop_type'] = Task.LoopType.Name(request.loop_type)
        if request.topics:
            fields['topics'] = 1
            query['topics'] = {
                '$all': [t.encode('utf-8') for t in request.topics]}

        col = Mongo.collection(gflags.FLAGS.collection)
        matched_docs = col.find(query, request.fields)

        response = SearchResponse()
        response.total_count = matched_docs.count()
        docs = matched_docs.sort(SEARCH_SORT).skip(request.offset).limit(
            request.count)
        for doc in docs:
            Mongo.doc_to_pb(doc, response.tasks.add())

        print 'WarehouseQuery.search returns {} results for query={}'.format(
            response.total_count, query)
        return response

    @staticmethod
    def get_task(task_id):
        """Get a task from DB according to task_id."""
        query = {'id': task_id}
        col = Mongo.collection(gflags.FLAGS.collection)
        doc = col.find_one(query)
        print 'WarehouseQuery.get_task with query={}'.format(query)
        ret = Task()
        if doc:
            Mongo.doc_to_pb(doc, ret)
        else:
            flask.flash('Error: Cannot find task with ID.')
        return ret


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    request = SearchRequest()
    request.count = 1

    response = WarehouseQuery.search(request)
    print response, '\n\n'

    task = WarehouseQuery.get_task(response.tasks[0].id)
    print task.id
