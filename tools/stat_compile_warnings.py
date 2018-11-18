#!/usr/bin/env python3

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
"""Stat Apollo compile warnings, to encourage clean code."""

import collections
import fileinput
import re

kTopN = 20

# Target pattern: "<file>:<row>:<col>: warning: <reason>"
pat = re.compile(r'(?P<file>[^:]+):(?P<row>[0-9]+):(?P<col>[0-9]+): warning: .*')

warnings_recorded = set()  # file:row:col
warnings = collections.defaultdict(int)  # {file: count}
try:
    for line in fileinput.input():
        # Parse line.
        matched = pat.search(line.strip())
        if not matched:
            continue
        file_path, row, col = (
            matched.group('file'), matched.group('row'), matched.group('col'))

        # Stat.
        warnings_id = '{}-{}-{}'.format(file_path, row, col)
        if warnings_id not in warnings_recorded:
            warnings_recorded.add(warnings_id)
            warnings[file_path] += 1
except:
    # The input program may crash or be stopped.
    pass

# No warnings found.
if len(warnings) == 0:
    exit()

rankings = sorted([(count, file_path) for file_path, count in warnings.items()],
                  reverse = True)

print('\n================ Top noisy files during this compile ================')
for ranking, (count, file_path) in enumerate(rankings[:kTopN]):
    print('#{}: {} with {} warnings'.format(ranking + 1, file_path, count))
print('To expose warnings by file, please run:')
print('./apollo.sh clean; ./apollo.sh build | grep ": warning: " | grep <file>')
print('======================================================================')
