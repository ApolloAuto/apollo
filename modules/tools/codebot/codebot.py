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

import argparse
import subprocess
import os
import glob
import time
from git_helper import GitHelper
from file_editor import FileEditor

g_args = None

g_apollo_root = os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "../../../")

git_helper = None


def glob_files(path, pattern):
    matches = []
    for root, dirname, filenames in os.walk(path):
        for filename in filenames:
            if str(filename).endswith(pattern):
                matches.append(os.path.join(root, filename))
    return matches


def clean_gflags():
    header_files = []
    modules = os.path.join(g_apollo_root, "modules")
    header_files.extend(glob_files(modules, ".h"))
    header_files.extend(glob_files(modules, ".hpp"))

    src_files = []
    src_files.extend(glob_files(modules, ".cpp"))
    src_files.extend(glob_files(modules, ".cc"))

    code_files = header_files + src_files
    flag_declares = {}
    flag_defines = {}
    flag_used = []
    for filename in code_files:
        if not os.path.isfile(filename):
            continue
        f = FileEditor(filename)
        for flag in f.declared_gflags():
            if flag not in flag_declares:
                flag_declares[flag] = []
            flag_declares[flag].append(filename)
        for flag in f.defined_gflags():
            if flag not in flag_defines:
                flag_defines[flag] = []
            flag_defines[flag].append(filename)
        flag_used.extend(f.used_gflags())

    flag_used = set(flag_used)

    for flag in flag_declares:
        if flag not in flag_used:
            for f in flag_declares[flag]:
                fhandle = FileEditor(f)
                fhandle.delete_gflag(flag)
                fhandle.save()
    for flag in flag_defines:
        if flag not in flag_used:
            for f in flag_defines[flag]:
                fhandle = FileEditor(f)
                fhandle.delete_gflag(flag)
                fhandle.save()


def format_recent_files():
    oneday = 24 * 60 * 60
    commits = git_helper.get_commit_since_date("codebot",
                                               int(time.time()) - oneday)
    if len(commits) == 0:
        return
    files = git_helper.get_changed_files_since_commit(commits[-1])
    for file_name in files:
        if not os.path.isfile(file_name):
            continue
        file_editor = FileEditor(file_name)
        file_editor.delete_doxygen_file()
        file_editor.save(format=True)


if __name__ == "__main__":
    git_helper = GitHelper(g_apollo_root, remote="upstream")
    #clean_gflags()
    format_recent_files()
