#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2021 Apollo Authors. All Rights Reserved.
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
usage: pull_request.py files pull_id
       pull_request.py diff  pull_id
"""

import argparse
import os

from github import Github

token = os.getenv('GITHUB_API_TOKEN')


def get_pull(pull_id):
    """
    Args:
        pull_id (int): Pull id.
    Returns:
        github.PullRequest.PullRequest
    """
    github = Github(token, timeout=30)
    repo = github.get_repo('ApolloAuto/apollo')
    pull = repo.get_pull(pull_id)

    return pull


def get_files(args):
    """
    Args:
        args (argparse.ArgumentParser().parse_args()): Arguments.
    Returns:
        None.
    """

    pull = get_pull(args.pull_id)

    for file in pull.get_files():
        if file.filename.startswith('modules/perception/') or \
            file.filename.startswith('modules/drivers/canbus/can_client/esd') or \
            file.filename.startswith('modules/localization/msf')or \
            file.filename.startswith('modules/planning/learning_based') or \
            file.filename.startswith('modules/map/pnc_map:cuda_util_test') or \
            file.filename.endswith('.sh'):
            continue
        else:
          print(file.filename)

def diff(args):
    """
    Args:
        args (argparse.ArgumentParser().parse_args()): Arguments.
    Returns:
        None.
    """

    pull = get_pull(args.pull_id)

    for file in pull.get_files():
        if file.filename.startswith('modules/perception/') or \
            file.filename.startswith('modules/drivers/canbus/can_client/esd') or \
            file.filename.startswith('modules/localization/msf')or \
            file.filename.startswith('modules/planning/learning_based')or \
            file.filename.startswith('modules/map/pnc_map:cuda_util_test')or \
            file.filename.endswith('.sh'):
            continue
        else:
          print('+++ {}'.format(file.filename))
          print(file.patch)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()
    files_parser = subparsers.add_parser('files')
    files_parser.add_argument('pull_id', type=int)
    files_parser.set_defaults(func=get_files)
    diff_parser = subparsers.add_parser('diff')
    diff_parser.add_argument('pull_id', type=int)
    diff_parser.set_defaults(func=diff)

    args = parser.parse_args()
    args.func(args)