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


gflags.DEFINE_string('mongo_collection_name', 'records',
                     'MongoDB collection name.')


class RecordImporter(object):
    """Import Apollo record files."""

    @staticmethod
    def Import(record_file):
        """Import one record."""
        parser = RecordParser(record_file)
        if not parser.ParseMeta():
            glog.error('Fail to parse record {}'.format(record_file))
            return

        parser.ParseMessages()
        doc = Mongo.pb_to_doc(parser.record)

        collection = Mongo.collection(gflags.FLAGS.mongo_collection_name)
        collection.replace_one({'path': parser.record.path}, doc, upsert=True)
        glog.info('Imported record {}'.format(record_file))


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    if len(sys.argv) > 0:
        RecordImporter.Import(sys.argv[-1])
