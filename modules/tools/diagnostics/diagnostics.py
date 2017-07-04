#!/usr/bin/env python

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
Real Time Plotting of planning and control
"""
import curses
import os
import random
import rospy
import threading
import traceback
from std_msgs.msg import String

from Message import Message
import batch_include

primitive = (int, str, bool, unicode)


class Diagnostics(object):
    """
    Plotter Class
    """

    def __init__(self, stdscr):
        self.stdscr = stdscr
        # topic
        self.META_DATA_FILE = os.path.join(
            os.path.dirname(__file__), 'meta.data')
        self.messages = []

        curses.init_pair(1, curses.COLOR_RED, curses.COLOR_BLACK)
        curses.init_pair(2, curses.COLOR_GREEN, curses.COLOR_BLACK)
        self.stdscr.nodelay(1)
        curses.curs_set(0)

        self.lock = threading.Lock()
        try:
            with open(self.META_DATA_FILE) as f:
                for line in f:
                    module_name, proto_name, topic, period = line.strip().split(
                        ' ')
                    if module_name == 'MODULE' or proto_name == '#' or\
                        topic == '#' or period == '#':
                        pass
                    else:
                        cur_message = Message(module_name, proto_name, topic,
                                              period, self.stdscr, self.lock)
                        self.messages.append(cur_message)
        except Exception as e:
            print '{0} open failed! with error: {1}'.format(
                self.META_DATA_FILE, e.message)
            exit(1)
        self.options = []
        self.selection = 0
        for i in range(len(self.messages)):
            position = [0, i]
            self.options.append(position)
        self.current_index = 0

        self.MENU = True

    def callback_timer(self, event):
        """
        Update Main Screen
        """
        if self.MENU:
            with self.lock:
                self.stdscr.clear()
                self.stdscr.addstr(0, 0, "Module", curses.A_BOLD)
                self.stdscr.addstr(0, 15, "Topic", curses.A_BOLD)
                self.stdscr.addstr(0, 50, "Period", curses.A_BOLD)
                self.stdscr.addstr(0, 60, "Max", curses.A_BOLD)
                self.stdscr.addstr(0, 70, "Min", curses.A_BOLD)
                self.stdscr.addstr(0, 80, "Delay", curses.A_BOLD)

                for idx in xrange(len(self.messages)):
                    lidx = idx + 1
                    if idx == self.selection:
                        self.stdscr.addstr(lidx, 0, self.messages[idx].name,
                                           curses.A_REVERSE)
                    else:
                        self.stdscr.addstr(lidx, 0, self.messages[idx].name)
                    self.stdscr.addstr(
                        lidx, 15, self.messages[idx].topic,
                        curses.color_pair(2 if self.messages[idx]
                                          .msg_received == True else 1))

                    self.stdscr.addstr(
                        lidx, 50,
                        "{0:.2f}".format(self.messages[idx].msg_interval))
                    self.stdscr.addstr(
                        lidx, 60, "{0:.2f}".format(self.messages[idx].msg_max))
                    self.stdscr.addstr(
                        lidx, 70, "{0:.2f}".format(self.messages[idx].msg_min))
                    self.stdscr.addstr(
                        lidx, 80,
                        "{0:.2f}".format(self.messages[idx].msg_delay))

                self.stdscr.refresh()


def main(stdscr):
    """
    Main function
    """
    rospy.init_node('adu_diagnostics_' + str(random.random()), anonymous=True)

    diag = Diagnostics(stdscr)
    sublist = []
    for msg in diag.messages:
        sublist.append(
            rospy.Subscriber(
                msg.topic, eval(msg.proto_name), msg.callback, queue_size=10))

    maintimercallback = rospy.Timer(rospy.Duration(0.05), diag.callback_timer)

    while True:
        with diag.lock:
            c = stdscr.getch()
            curses.flushinp()
            if c == ord('q') or c == 27:
                maintimercallback.shutdown()
                for sub in sublist:
                    sub.unregister()
                break
            if c == ord('b'):
                for sub in sublist:
                    sub.unregister()
            if c == curses.KEY_DOWN:
                if diag.MENU:
                    diag.selection = min((diag.selection + 1),
                                         len(diag.options) - 1)
                else:
                    diag.messages[diag.selection].key_down()
            elif c == curses.KEY_UP:
                if diag.MENU:
                    diag.selection = max((diag.selection - 1), 0)
                else:
                    diag.messages[diag.selection].key_up()
            elif c == curses.KEY_RIGHT:
                currentmsg = diag.messages[diag.selection]
                if diag.MENU:
                    diag.MENU = False
                    currentmsg.field.show = True
                    currentmsg.field.display_on_screen()
                else:
                    currentmsg.key_right()
            elif c == curses.KEY_LEFT:
                if not diag.MENU:
                    currentmsg = diag.messages[diag.selection]
                    if currentmsg.field.show:
                        currentmsg.field.show = False
                        diag.MENU = True
                    else:
                        currentmsg.key_left()
            elif c == ord('w'):
                currentmsg = diag.messages[diag.selection]
                if not diag.MENU:
                    currentmsg.index_incr()
            elif c == ord('s'):
                currentmsg = diag.messages[diag.selection]
                if not diag.MENU:
                    currentmsg.index_decr()
            elif c == ord('a'):
                currentmsg = diag.messages[diag.selection]
                if not diag.MENU:
                    currentmsg.index_begin()
            elif c == ord('d'):
                currentmsg = diag.messages[diag.selection]
                if not diag.MENU:
                    currentmsg.index_end()

        rospy.sleep(0.01)


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except Exception as e:
        tb = traceback.format_exc()
        print tb
