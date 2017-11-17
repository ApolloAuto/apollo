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
Data Sync.
"""

import os
import sys
import time
import pipes
import shutil
import logging
import datetime
import commands
import threading
import subprocess

import disk_handle

class DataSync(threading.Thread):
    """Sync data."""
    def __init__(self, recorder_manager):
        """Init"""
        threading.Thread.__init__(self)
        self.recorder_manager = recorder_manager
        self.conf_reader = recorder_manager.conf_reader

    def check_file(self, filename):
        """Check if a file is used by other process."""
        if os.path.exists(filename):
            cmd = "echo \"rootpass\n\"|sudo -S lsof -f -- " + filename
            process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
            out, err = process.communicate()
            errcode = process.returncode
            if errcode != 0:
                # file exists and is not being used.
                return 1
            else:
                # file exists and is being used.
                return 2
        else:
            return 3

    def do_sync(self, src, dst, backup=None, limit=102400, with_remove=False):
        """Sync data."""
        if not os.path.exists(src):
            logging.warn("%s do not exits.", src)
            return
        sync_src = src if src.endswith('/') else src + "/"
        sync_dst = self.recorder_manager.output_directory + "/" + dst + "/"
        cmd = "mkdir -p " + sync_dst \
                + " && echo \"rootpass\n\" |sudo -S /usr/bin/rsync " \
                + "-auvrtzopgP --bwlimit=" \
                + str(limit) \
                + " " \
                + sync_src \
                + " " \
                + sync_dst
        sync_process = subprocess.Popen(
                cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)
        out, err = sync_process.communicate()
        errcode = sync_process.returncode
        if errcode == 0:
            logging.info("Sync %s to %s successfully with \
                args=(backup=%s, limit=%s, with_remove=%s)", 
                sync_src, sync_dst, backup, str(limit), with_remove)
        else:
            logging.error(
                "Sync %s to %s failed, cmd=%s, stderr=%s, errorcode=%s",
                sync_src, sync_dst, cmd, err, errcode)
            return
        if not with_remove:
            return
        # backup and remove.
        cmd = "echo \"rootpass\n\"|sudo -S find " + sync_src + \
              " -mmin +1 -type f"
        process = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE)
        out, err = process.communicate()
        errcode = process.returncode
        logging.info("Find files in src=%s, stdout=%s, stderr=%s, errorcode=%s", 
            sync_src, out, err, errcode)
        if errcode != 0:
            logging.error(
                "Find files failed, cmd=%s, stdout=%s, stderr=%s, errorcode=%s", 
                cmd, out, err, errcode)
            return
        for f in out.splitlines():
            if not os.path.exists(os.path.abspath(f)):
                logging.warn("Not valid file, %s", f)
                continue
            errno = self.check_file(f)
            if errno == 2:
                logging.warn("%s is used by other process", f)
                continue
            elif errno == 3:
                logging.warn("%s is not exist", f)
                continue
            if os.path.islink(os.path.abspath(f)):
                logging.info("%s is symlink, ignore.", f)
                continue
            if backup is None:
                try:
                    os.remove(f)
                except Exception as e:
                    logging.error("Remove %s failed", f)
            else:
                sync_src_tmp = sync_src.rstrip('/')
                backup_dst = backup \
                           + "/" \
                           + os.path.split(
                               os.path.abspath(f))[0].replace(sync_src_tmp, '')
                cmd = "mkdir -p " + backup_dst \
                        + " && echo \"rootpass\n\" |sudo -S mv " \
                        + f \
                        + " " \
                        + backup_dst
                process = subprocess.Popen(
                        cmd,
                        shell=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.PIPE)
                out, err = process.communicate()
                errcode = process.returncode
                if errcode == 0:
                    logging.info(
                        "Sync %s to %s with remove source files successfully",
                        f, backup_dst)
                else:
                    logging.info("Sync @cmptnode %s to %s with remove failed, \
                        cmd=%s, stderr=%s, errorcode=%s",
                        f, backup_dst, cmd, err, errcode)

    def sync_data(self):
        """Sync data."""
        if not self.recorder_manager.sync_enable:
            return
        for data_type, data_args in self.conf_reader.task_data_args.items():
            if not data_args['if_record']:
                continue
            if data_args['record_method'] != "rsync":
                continue
            else:
                src = data_args.get('data_property').get('src')
                dst = data_args.get('data_property').get('dst')
                bwlimit = data_args.get('action_args').get('sync_bwlimit')
                if data_args['action_args']['with_remove']:
                    backup_dir = self.recorder_manager.backup_directory + "/" \
                                 + dst
                    self.do_sync(src, dst, backup=backup_dir, limit=bwlimit,
                                 with_remove=True)
                else:
                    self.do_sync(src, dst, backup=None, limit=bwlimit,
                                 with_remove=False)

    def clean_backup(self):
        """Clean backup dir."""
        backup = self.conf_reader.data_args.get('backup_path')
        backup_days = self.conf_reader.data_args.get('backup_keep_days')
        backup_max_size = self.conf_reader.data_args.get('backup_max_size')
        cmd = "cd " + backup + " && find . -maxdepth 1 -ctime +" + str(
            backup_days) + " |xargs -i rm -rf {}"
        ret, rst = commands.getstatusoutput(cmd)
        logging.info("cmd=%s, return=%s, result=%s", cmd, ret, rst)
        backup_size = disk_handle.get_folder_size(backup)
        while (backup_size > int(backup_max_size) * 1024 * 1024):
            cmd = "cd " + backup + " && find -type f -printf \'%p\n\' | " + \
                  "sort|head -n 5| xargs -i rm {}"
            try:
                ret, rst = commands.getstatusoutput(cmd)
                logging.info("cmd=%s, return=%s, result=%s", cmd, ret, rst)
            except Exception as e:
                logging.warn("cmd=%s, error=%s", cmd, str(e))
              
    def run(self):
        """Thread run from here."""
        now_time = datetime.datetime.now()
        period = datetime.timedelta(seconds=120)
        next_time = now_time + period
        while not self.recorder_manager.stop_signal:
            time.sleep(1)
            if not self.recorder_manager.sync_enable:
                continue
            iter_now_time = datetime.datetime.now()
            if iter_now_time >= next_time:
                self.clean_backup()
                self.sync_data()
                next_time = iter_now_time + period
                continue
        logging.info("Catch stop signal and begin to sync log")
        print("\nCatch stop signal and begin to sync log, "
              "it will take a few minuts, please wait!")
        self.sync_data()
        logging.info("Catch stop signal and sync log finished.")
        print("Sync data finished!")
