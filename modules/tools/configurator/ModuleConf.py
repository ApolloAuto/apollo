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
Message Handle
"""

import curses
import importlib
from curses import panel

from google.protobuf import text_format

APOLLO_ROOT = '../../../..'


class ModuleConf(object):
    """
    ModuleConf Class
    """

    def __init__(self, name, proto_file, proto_class, conf_file, stdscr):
        self.name = name
        self.proto_file = proto_file
        self.proto_class = proto_class
        self.conf_file = conf_file
        self.stdscr = stdscr
        self.found_conf = True

    def parse_from_file(self):
        mod = importlib.import_module(self.proto_file)
        self.proto = eval("mod." + self.proto_class)

        try:
            with open(APOLLO_ROOT + self.conf_file, 'r') as prototxt:
                text_format.Parse(prototxt.read(), self.proto)
        except:
            self.found_conf = False
        return

    def show(self):
        cornerx = 2
        cornery = 10
        self.field = Field(self.proto, self.stdscr, self.proto.DESCRIPTOR,
                           cornerx, cornery, self.proto, self.conf_file, False)
        self.field.process_input()


class Field(object):
    """
    Item in Message Class
    """

    def __init__(self, item, win, descriptor, cornerx, cornery, proto_root,
                 conf_file, is_repeated):
        self.item = item
        self.select = 0
        self.descriptor = descriptor
        self.index = 0
        self.basex = 2
        self.basey = 4
        self.cornerx = cornerx
        self.cornery = cornery
        self.proto_root = proto_root
        self.conf_file = conf_file
        self.is_repeated = is_repeated

        x, y = win.getmaxyx()
        self.win = curses.newwin(x - 3, y - 12, cornerx, cornery)
        self.win.clear()
        self.win.box()
        self.panel = curses.panel.new_panel(self.win)
        self.panel.top()
        curses.panel.update_panels()
        self.winy, self.winx = self.win.getmaxyx()
        self.setup()
        self.print_out()

    def setup(self):
        self.elelist = []
        if self.descriptor.containing_type is not None and \
                self.descriptor.label == self.descriptor.LABEL_REPEATED:
            printstring = self.descriptor.name + " [Index: " + str(
                self.index) + "]"
            self.win.addstr(1, 2, printstring, curses.A_BOLD)

            self.prefetch(self.item[self.index], self.descriptor, self.basex,
                          self.basey, [self.index])
        else:
            self.win.addstr(1, 2, self.descriptor.name, curses.A_BOLD)
            self.prefetch(self.item, self.descriptor, self.basex, self.basey,
                          [])

    def prefetch(self, entity, descriptor, row, col, descript_path):
        if descriptor.containing_type is None or \
           descriptor.type == descriptor.TYPE_MESSAGE:
            for descript, item in entity.ListFields():
                if row >= self.winy - 1:
                    if col >= (self.winx // 3) * 2:
                        return row, col
                    row = self.basex
                    col = col + self.winx // 3

                descript_path.append(descript.name)
                if descript.label == descript.LABEL_REPEATED:
                    ele = element(row, col, descript, item, self.cornerx,
                                  self.cornery, self.item,
                                  list(descript_path), True)
                    self.elelist.append(ele)
                elif descript.type == descript.TYPE_MESSAGE:
                    self.win.addstr(row, col, descript.name + ": ")
                    row, col = self.prefetch(item, descript, row + 1, col + 2,
                                             descript_path)
                    row -= 1
                    col -= 2
                else:
                    self.prefetch(item, descript, row, col, descript_path)
                row += 1
                descript_path.pop()
            return row, col

        ele = element(row, col, descriptor, entity, self.cornerx, self.cornery,
                      self.item, list(descript_path), False)
        self.elelist.append(ele)
        return row, col

    def print_out(self):
        """
        Display every element
        """
        for i in range(len(self.elelist)):
            self.elelist[i].print_element(self.win, True
                                          if i == self.select else False)
        self.win.refresh()

    def write_to_file(self):
        with open(APOLLO_ROOT + self.conf_file, 'w') as prototxt:
            prototxt.write(text_format.MessageToString(self.proto_root))

    def process_input(self):
        self.win.keypad(1)
        while True:
            user_input = self.win.getch()
            if user_input == curses.KEY_DOWN:
                self.select = min(self.select + 1, len(self.elelist) - 1)
            elif user_input == curses.KEY_UP:
                self.select = max(self.select - 1, 0)
            elif user_input == curses.KEY_LEFT:
                if self.is_repeated:
                    self.index = max(self.index - 1, 0)
                    self.setup()
            elif user_input == curses.KEY_RIGHT:
                if self.is_repeated:
                    self.index = min(self.index + 1, len(self.item) - 1)
                    self.setup()
            elif user_input == ord('q') or user_input == 27:
                del self.win
                del self.panel
                curses.panel.update_panels()
                return
            elif user_input == ord('\n'):
                if self.select < len(self.elelist):
                    if self.elelist[self.select].is_repeated:
                        proto = self.elelist[self.select].value
                        descript = self.elelist[self.select].descriptor
                        field = Field(proto, self.win, descript,
                                      self.cornerx + 2, self.cornery + 10,
                                      self.proto_root, self.conf_file, True)
                        field.process_input()
                    else:
                        self.elelist[self.select].edit()

            elif user_input == ord('w'):
                for ele in self.elelist:
                    ele.update_proto()
                self.write_to_file()

            self.print_out()


class element(object):

    def __init__(self, row, col, descriptor, value, cornerx, cornery,
                 proto_root, descript_path, is_repeated):
        self.row = row
        self.col = col
        self.descriptor = descriptor
        self.value = value
        self.modified = False
        self.cornerx = cornerx
        self.cornery = cornery
        self.value_length = 0
        self.proto_root = proto_root
        self.descript_path = descript_path
        self.is_repeated = is_repeated

    def print_element(self, win, highlight):
        win.addstr(self.row, self.col, self.descriptor.name + ": ")
        attr = curses.A_REVERSE if highlight else curses.A_NORMAL
        if self.modified:
            attr = attr | curses.color_pair(1)

        if self.is_repeated:
            printstring = "[Repeated Item: " + str(len(self.value)) + "]"
            win.addstr(printstring, attr)
        elif self.descriptor.type == self.descriptor.TYPE_ENUM:
            enum_type = self.descriptor.enum_type.values_by_number[
                self.value].name
            win.addstr(enum_type, attr)
            if len(enum_type) < self.value_length:
                win.addstr(' ' * (self.value_length - len(enum_type)))
            self.value_length = len(enum_type)
        else:
            printstr = str(self.value)
            win.addstr(printstr, attr)
            if len(printstr) < self.value_length:
                win.addstr(' ' * (self.value_length - len(printstr)))
            self.value_length = len(printstr)

    def update_proto(self):
        if not self.modified:
            return

        cur = self.proto_root
        for i in range(len(self.descript_path) - 1):
            if isinstance(self.descript_path[i], int):
                cur = cur[self.descript_path[i]]
            else:
                cur = getattr(cur, self.descript_path[i])
        if isinstance(self.descript_path[-1], int):
            cur[self.descript_path[-1]] = self.value
        else:
            setattr(cur, self.descript_path[-1], self.value)
        self.modified = False

    def get_input(self, options, select):
        for i in range(len(options)):
            self.win.addstr(i, 0, options[i], curses.A_REVERSE
                            if select == i else curses.A_NORMAL)
        self.win.refresh()
        self.win.keypad(1)
        while True:
            user_input = self.win.getch()
            if user_input == curses.KEY_DOWN:
                select = min(select + 1, len(options) - 1)
            elif user_input == curses.KEY_UP:
                select = max(select - 1, 0)
            elif user_input == 27:
                return select, False
            elif user_input == ord('\n'):
                return select, True
            for i in range(len(options)):
                self.win.addstr(i, 0, options[i], curses.A_REVERSE
                                if select == i else curses.A_NORMAL)
            self.win.refresh()

    def edit(self):
        x = self.row + self.cornerx
        y = self.col + self.cornery + len(self.descriptor.name) + 2
        if self.descriptor.type == self.descriptor.TYPE_ENUM:
            options = len(self.descriptor.enum_type.values)
            self.win = curses.newwin(options, 20, x, y)
        elif self.descriptor.type == self.descriptor.TYPE_BOOL:
            self.win = curses.newwin(2, 6, x, y)
        else:
            self.win = curses.newwin(1, 20, x, y)
        self.win.bkgd(' ', curses.color_pair(3))
        self.win.erase()
        self.panel = curses.panel.new_panel(self.win)
        self.panel.top()
        curses.panel.update_panels()
        self.win.refresh()

        if self.descriptor.type == self.descriptor.TYPE_ENUM:
            options = [value.name for value in self.descriptor.enum_type.values]
            value, modified = self.get_input(options, self.value)
            if modified and value != self.value:
                self.value = value
                self.modified = modified

        elif self.descriptor.type == self.descriptor.TYPE_BOOL:
            options = ["False", "True"]
            select = 1 if self.value else 0
            value, modified = self.get_input(options, select)
            value = True if value == 1 else False
            if modified and value != self.value:
                self.value = value
                self.modified = modified
        else:
            curses.echo()
            s = self.win.getstr(0, 0, 20)
            if len(s) == 0:
                pass
            elif self.descriptor.type == self.descriptor.TYPE_FLOAT or \
                    self.descriptor.type == self.descriptor.TYPE_DOUBLE:
                try:
                    value = float(s)
                    self.modified = True
                    self.value = value
                except ValueError:
                    pass
            elif self.descriptor.type == self.descriptor.TYPE_STRING:
                if s != self.value:
                    self.value = s
                    self.modified = True
            else:
                try:
                    value = int(s)
                    self.modified = True
                    self.value = value
                except ValueError:
                    pass
            curses.noecho()

        del self.win
        del self.panel
        curses.panel.update_panels()
