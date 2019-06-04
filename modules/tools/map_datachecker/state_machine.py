#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
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
import yaml
import time
import logging

class StateMachine:
    def __init__(self):
        self._script_path = os.path.dirname(os.path.realpath(__file__))
        self._state_machine = os.path.join(self._script_path, 'state_machine.yaml')
        self._state_cur = os.path.join(self._script_path, 'state_machine.info')
        self._new_machine_begin_line = "Created a new mechine"
        self._time_format = "%Y%m%d-%H:%M:%S"

    def _now(self):
        return time.strftime(self._time_format, time.localtime(time.time()))

    def _get_his_state_of_cur_machine(self):
        his_state = []
        if not os.path.exists(self._state_cur):
            return [0, his_state]
        with open(self._state_cur, 'r') as fp:
            lines = fp.readlines()
            state_count = len(lines)
            if state_count == 0:
                return [0, his_state]
            line_begin = -1
            for i in range(state_count - 1, -1, -1):
                if self._new_machine_begin_line in lines[i]:
                    line_begin = i
                    break
            if line_begin < 0:
                logging.error("find [%s] in file [%s] failed" % (self._new_machine_begin_line, self._state_cur))
                return [-1, his_state]
            for i in range(line_begin, state_count):
                his_state.append(lines[i].strip())
            return his_state

    def _load_state_machine(self):
        with open(self._state_machine, 'r') as fp:
            return yaml.load(fp.read())

    def _valid_state(self, state_machine, his_state, cur_state, cmd):
        if cur_state == 'eight_route' or cur_state == 'static_align':
            pass

    def state_check(self, state, cmd, note):
        [ret, his_state] = self._get_his_state_of_cur_machine()
        if ret != 0:
            logging.error('get_his_state_of_cur_machine failed')
            return -1
        if len(his_state) == 0:
            logging.info('there is no history state')
            return 0
        
        state_machine = self._load_state_machine()
        return self._valid_state(state_machine, his_state, state, cmd)

    def state_flow(self, state, cmd, note):
        with open(self._state_cur, 'a') as fp:
            fp.write('%s %s %s %s' % (self._now(), state, cmd, note))

    def state_abort(self, state, cmd, note):
        with open(self._state_cur, 'a') as fp:
            fp.write('%s %s abort abort' % (self._now(), state))
    
if __name__ == "__main__":
    sm = StateMachine()
    print sm.now()
