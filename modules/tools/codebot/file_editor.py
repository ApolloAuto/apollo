#!/usr/bin/env python

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

import os
import sys
import re
import subprocess

class FileEditor:
    define_gflags_pattern = re.compile("DEFINE_[a-z]+\(([a-z_]+),")
    declare_gflags_pattern = re.compile("DECLARE_[a-z]+\(([a-z_]+)\)")
    used_gflags_pattern = re.compile("FLAGS_([a-z_]+)")

    def __init__(self, filename):
        self.__filename = filename
        self.__ext = os.path.splitext(filename)[1]
        self.__dir = os.path.dirname(filename)
        self.__basename = os.path.basename(filename)
        with file(filename, "r") as fhandle:
            self.__content = fhandle.read()

    def save(self, format=True):
        with file(self.__filename, 'w') as fhandle:
            fhandle.write(self.__content)
        if not format:
            return

        if self.__basename == "BUILD":
            self.execute(['buildifier', self.__filename])
        elif self.__ext == ".cc" or self.__ext == ".h" or self.__ext == ".hpp":
            self.execute(['clang-format', '-i', '-style=Google', self.__filename])
        elif self.__ext == ".py":
            self.execute(['yapf', '-i', self.__filename])
        else:
            print "Skip formatting unkown file type: %s" % self.__filename

    def declared_gflags(self):
        return FileEditor.declare_gflags_pattern.findall(self.__content)

    def defined_gflags(self):
        return FileEditor.define_gflags_pattern.findall(self.__content)

    def delete_doxygen_file(self):
        self.__content = re.sub('^ \* @file %s$' % self.__basename, ' * @file', self.__content)

    def delete_gflag(self, flag):
        self.__content = re.sub(r'DEFINE_[a-z]+\(%s,\s*(.|\s)*?"\);' % flag, "", self.__content, re.MULTILINE)
        self.__content = re.sub(r'DECLARE_[a-z]+\(%s\);' % flag, "", self.__content, re.MULTILINE)
        self.__content = re.sub(r'--(no)?%s(=.*?)?' % flag, "", self.__content, re.MULTILINE)

    def used_gflags(self):
        return FileEditor.used_gflags_pattern.findall(self.__content)

    def execute(self, command):
        result = subprocess.call(command, stdout=subprocess.PIPE)
        return result

