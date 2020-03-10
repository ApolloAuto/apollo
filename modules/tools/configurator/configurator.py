#!/usr/bin/env python3

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
Tool to modify configuration files
"""
import curses
import os
import traceback
import six
from six.moves import xrange  # pylint: disable=redefined-builtin

from ModuleConf import ModuleConf

# Reduce ESC input delay
os.environ.setdefault('ESCDELAY', '25')


class Configurator(object):
    """
    Configurator Class
    """

    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.stdscr.box()
        # topic
        self.META_DATA_FILE = os.path.join(
            os.path.dirname(__file__), 'meta.data')
        self.messages = []

        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        curses.init_pair(3, curses.COLOR_RED, curses.COLOR_WHITE)
        curses.curs_set(0)

        self.moduleconf = []

        with open(self.META_DATA_FILE) as f:
            for line in f:
                module_name, proto_file, proto_class, conf_file = line.strip(
                ).split(' ')
                if module_name == 'MODULE' or proto_file == '#' or\
                        proto_class == "#" or conf_file == '#':
                    pass
                else:
                    moduleconf = ModuleConf(module_name, proto_file,
                                            proto_class, conf_file, self.stdscr)
                    moduleconf.parse_from_file()
                    self.moduleconf.append(moduleconf)

        self.select = 0
        self.winy, _ = stdscr.getmaxyx()

    def process_input(self):
        while True:
            user_input = self.stdscr.getch()
            if user_input == curses.KEY_DOWN:
                self.select = min((self.select + 1), len(self.moduleconf))
            elif user_input == curses.KEY_UP:
                self.select = max((self.select - 1), 0)
            elif user_input == ord('q') or user_input == 27:
                return
            elif user_input == ord('\n'):
                if self.select == len(self.moduleconf):
                    return
                else:
                    self.moduleconf[self.select].show()
            self.show()
            self.stdscr.refresh()

    def setup(self):
        self.stdscr.addstr(1, 2, "Module", curses.A_BOLD)
        self.stdscr.addstr(1, 10, "Proto", curses.A_BOLD)
        self.stdscr.addstr(1, 50, "Class", curses.A_BOLD)
        self.stdscr.addstr(1, 65, "Configuration File", curses.A_BOLD)

        for idx in range(len(self.moduleconf)):
            lidx = idx + 2
            if idx == self.select:
                self.stdscr.addstr(lidx, 2, self.moduleconf[idx].name,
                                   curses.A_REVERSE)
            else:
                self.stdscr.addstr(lidx, 2, self.moduleconf[idx].name)

            self.stdscr.addstr(lidx, 10, self.moduleconf[idx].proto_file)
            self.stdscr.addstr(lidx, 50, self.moduleconf[idx].proto_class)
            self.stdscr.addstr(lidx, 65, self.moduleconf[idx].conf_file,
                               curses.color_pair(1 if not self.moduleconf[idx]
                                                 .found_conf else 2))

        if self.select == len(self.moduleconf):
            self.stdscr.addstr(
                len(self.moduleconf) + 3, 2, "Exit", curses.A_REVERSE)
        else:
            self.stdscr.addstr(
                len(self.moduleconf) + 3, 2, "Exit", curses.A_BOLD)

        self.stdscr.addstr(self.winy - 9, 2,
                           "[Up][Down]: Choose different item")
        self.stdscr.addstr(self.winy - 8, 2, "[Enter] Select item/Make Change")
        self.stdscr.addstr(self.winy - 7, 2,
                           "[Left][Right]: Index through repeated item")
        self.stdscr.addstr(self.winy - 6, 2, "[ESC][q]: Close")
        self.stdscr.addstr(self.winy - 5, 2, "[w]: Write change to file")
        self.stdscr.addstr(self.winy - 4, 2, "Red", curses.color_pair(1))
        self.stdscr.addstr(": change made but NOT saved to file yet")
        self.stdscr.addstr(
            self.winy - 3, 2,
            "Change will be lost if panel closed, or changing repeated item index"
        )

    def show(self):
        """
        Update Main Screen
        """
        for idx in range(len(self.moduleconf)):
            lidx = idx + 2
            if idx == self.select:
                self.stdscr.addstr(lidx, 2, self.moduleconf[idx].name,
                                   curses.A_REVERSE)
            else:
                self.stdscr.addstr(lidx, 2, self.moduleconf[idx].name)

        if self.select == len(self.moduleconf):
            self.stdscr.addstr(
                len(self.moduleconf) + 3, 2, "Exit", curses.A_REVERSE)
        else:
            self.stdscr.addstr(
                len(self.moduleconf) + 3, 2, "Exit", curses.A_BOLD)


def main(stdscr):
    """
    Main function
    """
    conf = Configurator(stdscr)

    conf.setup()

    conf.show()

    conf.process_input()


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except Exception as e:
        tb = traceback.format_exc()
        print(tb)
