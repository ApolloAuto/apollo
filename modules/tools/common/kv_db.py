#!/usr/bin/env python2

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
"""KV DB utils."""
import sys

import gflags
import plyvel

gflags.DEFINE_string("kv_db_path", "/apollo/data/kv_db", "Path to KV DB.")


class KVDB(object):
    """KV DB helper functions."""

    @staticmethod
    def Put(key, value):
        """Put an entry into DB."""
        db = plyvel.DB(gflags.FLAGS.kv_db_path, create_if_missing=True)
        db.put(key, value)
        db.close()

    @staticmethod
    def Delete(key):
        """Delete a key from DB."""
        db = plyvel.DB(gflags.FLAGS.kv_db_path, create_if_missing=True)
        db.delete(key)
        db.close()

    @staticmethod
    def Get(key):
        """Get value of a key from DB."""
        db = plyvel.DB(gflags.FLAGS.kv_db_path, create_if_missing=True)
        value = db.get(key)
        db.close()
        return value

    @staticmethod
    def List():
        """List values from DB."""
        db = plyvel.DB(gflags.FLAGS.kv_db_path, create_if_missing=True)
        for key, value in db:
            print(key, ":", value)
        db.close()


if __name__ == '__main__':
    def _help():
        print """Usage:
            {0} put <key> <value>
            {0} del <key>
            {0} get <key>
            {0} list
        """.format(sys.argv[0])
        sys.exit(0)

    if len(sys.argv) < 2:
        _help()

    gflags.FLAGS(sys.argv)
    op = sys.argv[1]
    if len(sys.argv) >= 3:
        key = sys.argv[2]
    if op == 'put':
        KVDB.Put(key, sys.argv[3]) if len(sys.argv) == 4 else _help()
    elif op == 'del':
        KVDB.Delete(key)
    elif op == 'get':
        print KVDB.Get(key)
    elif op == 'list':
        KVDB.List()
    else:
        _help()
