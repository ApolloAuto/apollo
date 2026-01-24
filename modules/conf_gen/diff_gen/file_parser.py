#!/usr/bin/env python3

###############################################################################
# Copyright 2024 The Apollo Authors. All Rights Reserved.
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
File Parser
"""
import os
import re
import json
import yaml
from enum import Enum
from collections import OrderedDict

import modules.conf_gen.common.color_print as p
from modules.conf_gen.common import utils


class FileType(Enum):
    """ file type enum
    """
    UNKNOWN = 0
    PROTO_FILE = 1
    YAML_FILE = 2
    FLAGS_FILE = 3
    JSON_FILE = 4


FILE_TYPE_MAP = {
    '.*\.proto$': FileType.PROTO_FILE,
    '.*\.pb.txt$': FileType.PROTO_FILE,
    '.*\.flags?$': FileType.FLAGS_FILE,
    '.*flags?.conf$': FileType.FLAGS_FILE,
    '.*\.yaml': FileType.YAML_FILE,
    '.*\.yml': FileType.YAML_FILE,
    '.*\.json': FileType.JSON_FILE
}


class FileParser:
    """ File Parser
    """

    def __init__(self, ENV, file_path, diff_param):
        self.env = ENV
        self.file_path = file_path
        self.diff_param = diff_param

    def gen(self):
        """ diff gen
        """
        if not self.load():
            return False
        if not self.update_diff():
            return False
        return self.dump()

    def load(self):
        """ load file
        """
        file_path = os.path.join(self.env['tmp_dir'], self.file_path)
        if not os.path.isfile(file_path):
            p.error(f'diff_gen failed, {file_path} not found.')
            return False
        self.file_type = self.get_file_type(file_path)
        if self.file_type == FileType.JSON_FILE:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    self.data = json.load(f, object_pairs_hook=OrderedDict)
                    return True
            except Exception as e:
                p.error(f'diff_gen parse json file {self.file_path} error: {e}')
                return False
        elif self.file_type == FileType.YAML_FILE:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    self.data = yaml.safe_load(f)
                    return True
            except Exception as e:
                p.error(f'diff_gen parse yaml file {file_path} error: {e}')
                return False
        else:
            lines = []
            for line in open(file_path, 'r', encoding='utf-8').readlines():
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                line = line.split('#', 1)[0].strip()
                lines.extend(line.replace('{', '{\n').replace('}', '\n}').split('\n'))
            self.data = self.parse_lines(lines)
        return True

    def get_file_type(self, file_path):
        """ get file type
        """
        for pattern, file_type in FILE_TYPE_MAP.items():
            if re.match(pattern, file_path):
                return file_type
        else:
            return FileType.UNKNOWN

    def parse_lines(self, lines):
        """ load proto,flags file
        """
        data = OrderedDict()
        index = 0
        while index < len(lines):
            line = lines[index].strip()
            if not line or line.startswith('#'):
                index += 1
                continue
            line = line.split('#', 1)[0].strip()
            if line[:2] == '--':
                self.file_type = FileType.FLAGS_FILE
                # flags file
                splits = line[2:].split('=', 1)
                if len(splits) == 2:
                    data[splits[0].strip()] = splits[1].strip()
                else:
                    data[splits[0].strip()] = ''
            elif ':' in line:
                for pairs in line.split(','):
                    splits = pairs.split(':', 1)
                    key, value = splits[0].strip(), splits[1].strip()
                    if key in data:
                        if not isinstance(data[key], list):
                            data[key] = [data[key]]
                        data[key].append(value)
                    else:
                        data[key] = value
            elif '{' in line:
                key = line.split('{', 1)[0].strip()
                l, index = self.find_object('{', '}', lines, index + 1)
                if key in data:
                    if not isinstance(data[key], list):
                        data[key] = [data[key]]
                    data[key].append(self.parse_lines(l))
                else:
                    data[key] = self.parse_lines(l)
            elif '[' in line:
                key = line.split('[', 1)[0].strip()
                data[key] = []
                l, index = self.find_object('[', ']', lines, index + 1)
                contents = '\n'.join(l)
                for content in contents.split(','):
                    content = content.strip().strip('{},')
                    data[key].append(self.parse_lines(content.split('\n')))
            index += 1
        return data

    def find_object(self, start_char, end_char, lines, index):
        """ find closure char
        """
        l = []
        start_char_count = 0
        while index < len(lines):
            line = lines[index].strip()
            if not line or line.startswith('#'):
                index += 1
                continue
            line = line.split('#', 1)[0].strip()
            if start_char in line:
                start_char_count += 1
                l.append(line)
            elif end_char in line:
                if start_char_count == 0:
                    content = line.split(end_char, 1)[0].strip()
                    if content:
                        l.append(content)
                    return l, index
                else:
                    start_char_count -= 1
                    l.append(line)
            else:
                l.append(line)
            index += 1
        return l, index

    def update_diff(self):
        """ update diff params
        """
        if not isinstance(self.data, dict):
            p.error(f'file {self.file_path} does not support diff gen')
            return False

        def find_key(keys, key_type):
            data = self.data
            for key in keys:
                index = -1
                if key[-1] == ']':
                    index = key[:-1].rsplit('[')[-1]
                    if not index.isdigit():
                        return False, None
                    index = int(index)
                    if index < 0:
                        return False, None
                    key = key.split('[', 1)[0]
                if not isinstance(data, dict):
                    return False, None
                if key not in data:
                    return False, None
                if index >= 0:
                    if not isinstance(data[key], list):
                        return False, None
                    data = data[key][index]
                else:
                    data = data[key]
            if not isinstance(data, key_type):
                return False, None
            return True, data

        def add(keys, value):
            succ, data = find_key(keys[:-1], dict)
            if not succ:
                return False
            data[keys[-1]] = value
            return True

        def update(keys, value):
            succ, data = find_key(keys[:-1], dict)
            if not succ:
                return False
            k = keys[-1]
            v = None
            index = 0
            is_list = '[' in k
            if is_list:
                index = k[:-1].rsplit('[')[-1]
                if not index.isdigit():
                    return False
                index = int(index)
                k = k.split('[', 1)[0]
                if k not in data or not isinstance(data[k], list):
                    return False
                if len(data[k]) < index + 1:
                    return False
                v = data[k][index]
            else:
                if k not in data:
                    return False
                v = data[k]
            if v and isinstance(v, str) and v.count('"') == 2:
                if str(value).count('"') != 2:
                    value = f'"{value}"'
            if is_list:
                data[k][index] = value
            else:
                data[k] = value
            return True

        def delete(keys):
            succ, data = find_key(keys[:-1], dict)
            if not succ:
                return False
            k = keys[-1]
            v = None
            index = 0
            is_list = '[' in k
            if is_list:
                index = k[:-1].rsplit('[')[-1]
                if not index.isdigit():
                    return False
                index = int(index)
                k = k.split('[', 1)[0]
                if k not in data or not isinstance(data[k], list):
                    return False
                if k not in data or not isinstance(data[k], list):
                    return False
                if len(data[k]) < index + 1:
                    return False
                del data[k][index]
            else:
                if k not in data:
                    return False
                del data[k]
            return True

        def insert(keys, value):
            succ, data = find_key(keys[:-1], dict)
            if not succ:
                return False
            key = keys[-1]
            index = key[:-1].rsplit('[')[-1]
            if not index.isdigit():
                return False
            index = int(index)
            if index < 0:
                return False
            key = key.split('[', 1)[0]
            if key not in data:
                return False
            if not isinstance(data[key], list):
                data[key] = [data[key]]
            if len(data[key]) < index:
                return False
            data[key].insert(index, value)
            return True

        func_map = {
            'add': add,
            'update': update,
            'insert': insert,
        }
        for action, params in self.diff_param.items():
            for param in params:
                if action == 'delete':
                    if not delete(param.split('.')):
                        p.error(f'diff gen parse file {self.file_path} error: {action} {param}')
                        return False
                else:
                    for key_path, value in param.items():
                        keys = key_path.split('.')
                        if not func_map[action](keys, value):
                            p.error(f'diff gen parse file {self.file_path} error: {action} {key_path}')
                            return False
        return True

    def dump(self):
        """ dump file
        """
        target_file = os.path.join(self.env['tmp_dir'], self.file_path)
        utils.ensure_dir(target_file.rsplit('/', 1)[0])
        with open(target_file, 'w', encoding='utf-8') as f:
            if self.file_type == FileType.YAML_FILE:
                yaml.dump(self.data, f, sort_keys=False)
            elif self.file_type == FileType.JSON_FILE:
                json.dump(self.data, f)
            elif self.file_type == FileType.FLAGS_FILE:
                for key, value in self.data.items():
                    if value == '':
                        f.write(f'--{key}\n')
                    else:
                        if isinstance(value, bool):
                            value = 'true' if value else 'false'
                        f.write(f'--{key}={value}\n')
            else:
                for key, value in self.data.items():
                    f.write(self.dump_proto(key, value))
        p.info(f'generate diff conf: {self.file_path}')
        return True

    def dump_proto(self, key, value, indent=0):
        """ dump proto
        """
        content = ''
        if isinstance(value, list):
            for v in value:
                if isinstance(v, (dict, OrderedDict)):
                    content += ' ' * indent + key + ' {\n'
                    for k_, v_ in v.items():
                        content += self.dump_proto(k_, v_, indent + 2)
                    content += ' ' * indent + '}\n'
                else:
                    if isinstance(v, bool):
                        v = 'true' if v else 'false'
                    content += ' ' * indent + f'{key}: {v}\n'
            return content
        if isinstance(value, (dict, OrderedDict)):
            content += ' ' * indent + key + ' {\n'
            for k, v in value.items():
                content += self.dump_proto(k, v, indent + 2)
            content += ' ' * indent + '}\n'
            return content
        if isinstance(value, bool):
            value = 'true' if value else 'false'
        return ' ' * indent + f'{key}: {value}\n'
