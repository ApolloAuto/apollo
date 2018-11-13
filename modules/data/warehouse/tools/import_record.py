#!/usr/bin/env python
# -*- coding: UTF-8-*-
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
Import Apollo record into a MongoDB.

Use as command tool: import_record.py <record>
Use as util lib:     RecordImporter.Import(<record>)
"""

import os
import sys

import gflags
import glog

from mongo_util import Mongo
from parse_record import RecordParser


gflags.DEFINE_string('collection', 'record', 'MongoDB collection name.')

gflags.DEFINE_string('import_mode', 'insert',
                     'Strategy when importing record to MongoDB:'
                     'insert: Insert new records only; '
                     'update: Update existing records only; '
                     'replace: Insert new or replace existing records.')


class RecordImporter(object):
    """
    Import Apollo record files.

    See https://api.mongodb.com/python/current/api/pymongo/collection.html for
    MongoDB collection document.
    As we used some advanced features, pymongo >= 3.0 is required.
    """

    @staticmethod
    def Import(record_file):
        """Import one record."""
        parser = RecordParser(record_file)
        if not parser.ParseMeta():
            glog.error('Fail to parse record {}'.format(record_file))
            return
        record_id = parser.record.id
        query = {'id': record_id}

        # Check if the record has been imported.
        collection = Mongo.collection(gflags.FLAGS.collection)
        mode = gflags.FLAGS.import_mode
        if mode == 'insert' and collection.find_one(query, {'_id': 1}):
            glog.info('Skip duplicate record with id {}'.format(record_id))
            return

        # Continue parsing.
        parser.ParseMessages()
        doc = Mongo.pb_to_doc(parser.record)

        # Operate MongoDB.
        if mode == 'insert':
            collection.insert_one(doc)
        elif mode == 'update':
            collection.update_one(query, {'$set': doc})
        elif mode == 'replace':
            collection.replace_one(query, doc, upsert=True)
        glog.info('Imported record {}'.format(record_id))


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    if len(sys.argv) > 0:
        RecordImporter.Import(sys.argv[-1])
