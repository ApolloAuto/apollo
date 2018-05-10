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
"""
MongoDB util.

Requirements: pymongo
"""
import os
import sys

import gflags
import google.protobuf.json_format as json_format
import pymongo


gflags.DEFINE_string('mongo_host', '127.0.0.1', 'MongoDB host ip.')
gflags.DEFINE_integer('mongo_port', 27017, 'MongoDB port.')
gflags.DEFINE_string('mongo_db_name', 'apollo', 'MongoDB db name.')
gflags.DEFINE_string('mongo_user', None, 'MongoDB user (optional).')
gflags.DEFINE_string('mongo_pass', None, 'MongoDB password (optional).')


class Mongo(object):
    """MongoDB util"""

    @staticmethod
    def db():
        """Connect to MongoDB instance."""
        G = gflags.FLAGS
        client = pymongo.MongoClient(G.mongo_host, G.mongo_port)
        db = client[G.mongo_db_name]
        if G.mongo_user and G.mongo_pass:
            db.authenticate(G.mongo_user, G.mongo_pass)
        return db

    @staticmethod
    def collection(collection_name):
        """
        Get collection handler. To use it, please refer
        https://api.mongodb.com/python/current/api/pymongo/collection.html
        """
        return Mongo.db()[collection_name]

    @staticmethod
    def pb_to_doc(pb):
        """Convert proto to mongo document."""
        including_default_value_fields = False
        preserving_proto_field_name = True
        return json_format.MessageToDict(pb, including_default_value_fields,
                                         preserving_proto_field_name)

    @staticmethod
    def doc_to_pb(doc, pb):
        """Convert mongo document to proto."""
        ignore_unknown_fields = True
        return json_format.ParseDict(doc, pb, ignore_unknown_fields)


if __name__ == '__main__':
    gflags.FLAGS(sys.argv)
    print Mongo.db().collection_names()
