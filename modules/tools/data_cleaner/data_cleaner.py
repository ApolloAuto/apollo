#!/usr/bin/env python3

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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
Data Cleaner: auto clean record and log
"""

import argparse
from datetime import datetime, timedelta
import logging
import os
import re
import signal
import sys
import time
import yaml

from modules.tools.common.logger import Logger

# TODO(all): hard-coded path temporarily. Better approach needed.
APOLLO_ROOT = "/apollo"
DEFAULT_CLEAN_INTERVAL = 60 # in seconds

Logger.config(
    log_file=os.path.join(APOLLO_ROOT, 'data/log/data_cleaner.log'),
    use_stdout=True,
    log_level=logging.DEBUG)
logger = Logger.get_logger("data_cleaner")


def quit(signum, _):
    """ quit
    """
    print('quit by signum: %s' % signum)
    sys.exit()


class FileObject:
    """ FileObject
    """
    def __init__(self, path, mtime, size):
        self.path = path
        self.mtime = mtime
        self.size = size


class DataCleaner:
    """ Data Cleaner Class
    """
    def __init__(self, config):
        """
        Params:
            config: config object
        """
        self.config = config

    def run(self):
        """ main run method
        """
        interval = self.config.get('clean_interval_in_secs') or DEFAULT_CLEAN_INTERVAL
        signal.signal(signal.SIGINT, quit)                                
        signal.signal(signal.SIGTERM, quit)
        while True:
            for strategy in self.config['strategies']:
                self.clean(strategy)
            time.sleep(interval)

    def clean(self, strategy):
        """ clean by each config item
        """
        logger.info("begin clean by config item [%s].", strategy['name'])
        expired_time_in_hours = strategy.get('expired_time_in_hours')
        usage_threshold_in_gb = strategy.get('usage_threshold_in_gb')
        # get all files by dirs
        total_size, file_list = self.get_all_files_by_dirs(strategy['dirs'], strategy.get('patterns'))
        index = 0
        if expired_time_in_hours:
            reserved_timestamp = (datetime.now() - timedelta(hours=expired_time_in_hours)).timestamp()
            for f in file_list:
                if f.mtime > reserved_timestamp:
                    break
                index += 1
                if self.remove(f.path):
                    total_size -= f.size
        if usage_threshold_in_gb:
            threshold = usage_threshold_in_gb * 1024 * 1024 * 1024
            while total_size > threshold and index < len(file_list):
                if self.remove(file_list[index].path):
                    total_size -= file_list[index].size
                index += 1
            if total_size > threshold:
                logger.warn("clean by item [%s] dirs [%s] cannot reach the usage threshold %sGB.",
                            strategy['name'], ",".join(strategy['dirs']), usage_threshold_in_gb)
        logger.info("finish clean by config item [%s].", strategy['name'])

    def get_all_files_by_dirs(self, dirs, patterns=[]):
        """ get all files by dir list
        Returns:
            (total size in bytes, FileObject List)
        """
        total_size = 0
        file_list = []
        progs = []
        if patterns:
            progs = [re.compile(pattern) for pattern in patterns]
        for d in dirs:
            for root, _, files in os.walk(d):
                for name in files:
                    filename = os.path.join(root, name)
                    if os.path.islink(filename):
                        continue
                    mtime = os.path.getmtime(filename)
                    size = os.path.getsize(filename)
                    total_size += size
                    if progs:
                        for prog in progs:
                            if prog.search(name):
                                break
                        else:
                            continue
                    file_list.append(FileObject(filename, mtime, size))
                    
        return (total_size, sorted(file_list, key=lambda f: f.mtime))

    def remove(self, file_path):
        """ remove file by file path
        """
        try:
            os.remove(file_path)
        except Exception as e:
            logger.error("remove file [%s] error: %s", file_path, e)
            return False
        logger.info("remove file [%s] successed.", file_path)
        return True


def validate_config(config):
    """ config Vilidator
    Params:
        config: config object
    Returns:
        error message or None
    """
    if not config:
        return "config is empty"
    if not isinstance(config, dict):
        return "config is not dict"
    clean_interval_in_secs = config.get('clean_interval_in_secs')
    if clean_interval_in_secs is not None:
        if not isinstance(clean_interval_in_secs, int):
            return 'clean_interval_in_secs must be a number'
        if clean_interval_in_secs < 1:
            return 'clean_interval_in_secs must be greater than 1'
    strategies = config.get('strategies')
    if strategies is None:
        return 'strategies is requried'
    if not isinstance(strategies, list):
        return 'strategies must be a list'
    for s in strategies:
        if not isinstance(s, dict):
            return "each config item should be a dict."
        if 'name' not in s or not isinstance(s['name'], str):
            return "name string field is require"
        if 'dirs' not in s or not isinstance(s['dirs'], list):
            return "item [%s] dirs missing" % s["name"]
        for d in s['dirs']:
            if not isinstance(d, str):
                return "item [%s] dir must be string" % s["name"]
        expired_time_in_hours = s.get('expired_time_in_hours')
        usage_threshold_in_gb = s.get('usage_threshold_in_gb')
        if not expired_time_in_hours and not usage_threshold_in_gb:
            return 'either expired_time_in_hours or usage_threshold_in_gb is required'
        if expired_time_in_hours:
            if not isinstance(expired_time_in_hours, int) or expired_time_in_hours < 1:
                return 'expired_time_in_hours [%s] is invalid.' % expired_time_in_hours
        if usage_threshold_in_gb:
            if not isinstance(usage_threshold_in_gb, int) or usage_threshold_in_gb < 1:
                return 'usage_threshold_in_gb [%s] is invalid.' % usage_threshold_in_gb
        
        patterns = s.get('patterns')
        if patterns:
            if not isinstance(patterns, list):
                return 'patterns must be a list'
            for pattern in patterns:
                if not isinstance(pattern, str):
                    return 'pattern [%s] is not a string' % pattern
                try:
                    re.compile(pattern)
                except Exception as e:
                    return 'pattern [%s] is invalid: %s' % (pattern, e)
    return None


def main():
    """ Main Method
    """
    parser = argparse.ArgumentParser(description='Data Cleaner Tool')
    parser.add_argument('-c', '--config', help='yaml config file path', required=True)
    args = parser.parse_args()

    # check and load config
    if not os.path.exists(args.config):
        logger.error("config [%s] not exist!", args.config)
        sys.exit(-1)
    try:
        with open(args.config, 'r', encoding='utf8') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        logger.error("load config [%s] error: %s", args.config, e)
        sys.exit(-1)
    error_message = validate_config(config)
    if error_message:
        logger.error("config format error: %s", error_message)
        sys.exit(-1)
    logger.debug("read config: %s", config)
    
    dc = DataCleaner(config)
    dc.run()


if __name__ == '__main__':
    main()
