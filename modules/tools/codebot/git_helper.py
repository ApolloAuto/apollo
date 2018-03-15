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
import git
import subprocess

class GitHelper:
    def __init__(self, path, remote):
        self.__path = path
        self.__repo = git.Repo(path)
        self.__remote = remote

    def execute(self, command):
        output = subprocess.check_output(command)
        return output

    def current_branch(self):
        return self.__repo.active_branch

    def find_branch(self, branch_name):
        return self.__repo.branches[branch_name]

    def rebase_with_remote(self):
        remote = self.find_remote(self.__remote)
        remote.fetch()
        self.execute(["git", "pull", "--rebase", "-Xtheirs", self.__remote, "master"])

    def find_remote(self, remote_name):
        for remote in self.__repo.remotes:
            if remote.name == remote_name:
                return remote
        print("Remote name %s not found" % name)

    def reset_to_remote(self):
        self.__repo.git.reset("--hard", "%s/master" % (self.__remote/master))

    def get_nth_commit_from_branch(self, branch, n):
        i = 0
        for commit in self.__repo.iter_commits(rev=branch):
            if i == n:
                return commit
            else:
                i += 1

    def get_all_commits_since(self, branch, last_commit):
        updated_commits = []
        for commit in self.__repo.iter_commits(rev=branch):
            updated_commits.append(commit)
            if commit.hexsha == last_commit.hexsha:
                break
        return updated_commits

    def get_commit_since_date(self, branch, unix_time):
        commits = []
        for commit in self.__repo.iter_commits(rev=branch):
            if commit.committed_date > unix_time:
                commits.append(commit)
        return commits


    def get_changed_files_since_commit(self, commit):
        output=self.execute(['git', 'diff', '--name-only', '%s..HEAD' % commit.hexsha])
        files = []
        for f in output.split("\n"):
            if len(f.strip()) == 0:
                continue
            print f
            files.append(os.path.join(self.__path, f))
        return files

