#!/usr/bin/env python3

###############################################################################
# Copyright 2020 The Apollo Authors. All Rights Reserved.
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
A plug-in that looks for similar commands when the input command has
spelling mistakes.
"""

import argparse

TAB = " " * 4


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--command', required=True)
    parser.add_argument('--available', type=str)
    parser.add_argument('--helpmsg', default="")

    return parser


def similar_words(word):
    """
    return a set with spelling1 distance alternative spellings

    based on http://norvig.com/spell-correct.html
    """
    alphabet = 'abcdefghijklmnopqrstuvwxyz-_0123456789'
    s = [(word[:i], word[i:]) for i in range(len(word) + 1)]
    deletes = [a + b[1:] for a, b in s if b]
    transposes = [a + b[1] + b[0] + b[2:] for a, b in s if len(b) > 1]
    replaces = [a + c + b[1:] for a, b in s for c in alphabet if b]
    inserts = [a + c + b for a, b in s for c in alphabet]
    return set(deletes + transposes + replaces + inserts)


class CommandChecker(object):
    def __init__(self, command, available_commands, name, help_msg=""):
        self.min_len = 2
        self.max_len = 256
        self.command = command
        self.available_commands = available_commands
        self.alternative_commands = []
        self.name = name
        self.help_message = help_msg

    def spelling_suggestions(self):
        """ try to correct the spelling """
        if not (self.min_len <= len(self.command) <= self.max_len):
            return
        for w in similar_words(self.command):
            for command in self.available_commands:
                if w == command:
                    self.alternative_commands.append(command)

    def print_spelling_suggestions(self, max_alt=15):
        """ print spelling suggestions """
        num_alternatives = len(self.alternative_commands)
        if num_alternatives > max_alt:
            print(
                'Error: unknown command "{}", but there are {} similar ones.'.
                format(self.command, num_alternatives))
            return
        elif num_alternatives > 0:
            print('Error: unknown command "{}" for "{}"\n'.format(
                self.command, self.name))
            print('Did you mean the following?')
            for command in self.alternative_commands:
                print('{}{}'.format(TAB * 2, command))
            print('')

    def advise(self):
        """ give advice """
        self.spelling_suggestions()
        if len(self.alternative_commands) > 0:
            self.print_spelling_suggestions()
        else:
            print('Error: unknown command "{}"\n'.format(self.command))

        if len(self.help_message) > 0:
            print(self.help_message)


def main():
    parser = get_parser()
    args = parser.parse_args()
    auto_checker = CommandChecker(
        args.command,
        args.available.split(),
        args.name,
        args.helpmsg,
    )
    auto_checker.advise()


if __name__ == "__main__":
    main()
