#!/usr/bin/env python3
# ****************************************************************************
# Copyright 2018 The Apollo Authors. All Rights Reserved.

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ****************************************************************************

import argparse
import atexit
import logging
import os
import os.path
import signal
import subprocess
import sys
import time
import threading
from datetime import datetime

import xml.etree.ElementTree as ET


g_binary_name = 'mainboard'
g_script_name = os.path.basename(sys.argv[0]).split(".")[0]
g_process_pid = os.getpid()
g_process_name = g_script_name + "_" + str(g_process_pid)
g_default_respawn_limit = 3

launch_path = os.getenv('APOLLO_LAUNCH_PATH') or '/apollo'
force_stop_timeout_secs = 3

"""
colorful logging
"""
BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = list(range(8))
RESET_SEQ = "\033[0m"
COLOR_SEQ = "\033[1;%dm"
BOLD_SEQ = "\033[1m"

COLORS = {
    'INFO': GREEN,
    'WARNING': YELLOW,
    'DEBUG': BLUE,
    'ERROR': RED,
    'CRITICAL': YELLOW
}

logger = logging.Logger(__name__)

# file logger
workspace_path = os.getenv('APOLLO_ENV_WORKROOT') or '/apollo'
log_dir_path = os.path.join(workspace_path, 'data/log')
if not os.path.exists(log_dir_path):
    os.makedirs(log_dir_path)
log_name = 'cyber_launch.log.INFO.{}.{}'.format(
    datetime.now().strftime("%Y%m%d-%H%M%S"), str(g_process_pid))
log_file_path = os.path.join(log_dir_path, log_name)
file_formater = logging.Formatter(
    '%(levelname)s [%(asctime)s] %(lineno)s: %(message)s')
file_hd = logging.FileHandler(filename=log_file_path)
file_hd.setFormatter(file_formater)
file_hd.setLevel(logging.INFO)
logger.addHandler(file_hd)
link_log_file = os.path.join(log_dir_path, 'cyber_launch.INFO')
try:
    if os.path.exists(link_log_file) or os.path.islink(link_log_file):
        os.remove(link_log_file)
    os.symlink(log_name, link_log_file)
except Exception as e:
    # Maybe mulitple process create link, ignore it
    pass


class ColoredFormatter(logging.Formatter):

    def __init__(self, msg):
        logging.Formatter.__init__(self, msg)

    def format(self, record):
        levelname = record.levelname
        if levelname in COLORS:
            if levelname == 'DEBUG':
                record.levelname = COLOR_SEQ % (30 + COLORS[levelname]) + \
                    record.msg.split('#')[0] + RESET_SEQ
                record.msg = COLOR_SEQ % (30 + COLORS[levelname]) + \
                    record.msg.split('#')[-1] + RESET_SEQ
            else:
                record.levelname = COLOR_SEQ % (30 + COLORS[levelname]) + \
                    g_process_name + RESET_SEQ
                record.msg = COLOR_SEQ % (30 + COLORS[levelname]) + levelname + \
                    " " + record.msg.split('#')[-1] + RESET_SEQ
        return logging.Formatter.format(self, record)


color_formatter = ColoredFormatter("[%(levelname)-18s] %(message)s")
console = logging.StreamHandler()
console.setFormatter(color_formatter)
logger.addHandler(console)


def exit_handler():
    stop()
    logger.info('cyber_launch exit.')


atexit.register(exit_handler)


def singleton(cls):
    instances = {}

    def getinstance(*args, **kwargs):
        if cls not in instances:
            instances[cls] = cls(*args, **kwargs)
        return instances[cls]
    return getinstance


def module_monitor(mod):
    while True:
        line = mod.popen.stdout.readline()
        if line:
            logger.debug('%s# %s' %
                         (mod.name, line.decode('utf8').strip('\n')))
            continue
        time.sleep(0.01)


class ProcessWrapper(object):

    def __init__(self, binary_path, dag_num, dag_list, plugin_list,
                 process_name, process_type, sched_name, extra_args_list,
                 exception_handler='', respawn_limit=g_default_respawn_limit,
                 cpu_profile_file='', mem_profile_file='', nice=0):
        self.time_of_death = None
        self.started = False
        self.cpu_profile = False
        self.mem_profile = False
        self.binary_path = binary_path
        self.dag_num = dag_num
        self.dag_list = dag_list
        self.plugin_list = plugin_list
        self.name = process_name
        self.sched_name = sched_name
        self.extra_args_list = extra_args_list
        self.process_type = process_type
        self.popen = None
        self.exit_code = None
        self.args = []
        self.pid = -1
        self.exception_handler = exception_handler
        self.respawn_limit = respawn_limit
        self.respawn_cnt = 0
        self.cpu_profile_file = cpu_profile_file
        self.mem_profile_file = mem_profile_file
        if self.cpu_profile_file != '':
            self.cpu_profile = True
        if self.mem_profile_file != '':
            self.mem_profile = True
        self.nice = nice

    def wait(self, timeout_secs=None):
        """
        Waiting for process terminal by call Popen.wait function
        """
        if self.started:
            if timeout_secs:
                self.popen.wait(timeout_secs)
            else:
                self.popen.wait()

    def kill(self):
        """
        Force kill process
        """
        if self.started:
            self.popen.kill()

    def start(self):
        """
        Start a manager in process name
        """
        if self.process_type == 'binary':
            args_list = self.name.split()
        else:
            args_list = [self.binary_path]
            for i in self.dag_list:
                args_list.append('-d')
                args_list.append(i)
            for i in self.plugin_list:
                args_list.append('--plugin')
                args_list.append(i)
            if len(self.name) != 0:
                args_list.append('-p')
                args_list.append(self.name)
            if len(self.sched_name) != 0:
                args_list.append('-s')
                args_list.append(self.sched_name)
            if len(self.extra_args_list) != 0:
                args_list.extend(self.extra_args_list)
            if self.cpu_profile:
                args_list.append('-c')
                args_list.append('-o')
                args_list.append(self.cpu_profile_file)
            if self.mem_profile:
                args_list.append('-H')
                args_list.append('-O')
                args_list.append(self.mem_profile_file)

        self.args = args_list

        try:
            self.popen = subprocess.Popen(args_list, stdout=None,
                                          stderr=subprocess.STDOUT)
        except Exception as err:
            logger.error('Subprocess Popen exception: ' + str(err))
            return 2
        else:
            if self.popen.pid == 0 or self.popen.returncode is not None:
                logger.error('Start process [%s] failed.', self.name)
                return 2

        # th = threading.Thread(target=module_monitor, args=(self, ))
        # th.setDaemon(True)
        # th.start()
        self.started = True
        self.pid = self.popen.pid
        if self.nice != 0:
            sudo_check = subprocess.run(
                ['sudo', '-n', 'true'], stdout=None, stderr=None)
            if sudo_check.returncode == 0:
                subprocess.run(
                    ["sudo", "renice", "-n", str(self.nice), "-p", str(self.pid)],
                    stdout=None, stderr=subprocess.STDOUT)
            else:
                logger.error(
                    'Can not renice because users do not have sudo privileges')
        logger.info(
            'Start process [%s] successfully. pid: %d', self.name, self.popen.pid)
        logger.info('-' * 120)
        return 0

    def is_alive(self):
        """
        Check the process if is still running
        @return: True if process is still running
        @rtype: bool
        """
        if not self.started:
            return False

        if self.popen is None:
            if self.time_of_death is None:
                self.time_of_death = time.time()
            return False

        self.exit_code = self.popen.poll()
        if self.exit_code is not None:
            if self.time_of_death is None:
                self.time_of_death = time.time()
            return False
        return True

    def get_exit_state(self):
        """
        @return: description of exit state
        @rtype: str
        """
        if self.popen.returncode is None:
            pass
        elif self.popen.returncode != 0:
            output = 'Process [%s] has died [pid %s, exit code %s, cmd %s].' % \
                     (self.name, self.pid, self.exit_code, ' '.join(self.args))
            logger.error(output)
        else:
            output = 'Process [%s] has finished. [pid %s, cmd %s].' % \
                     (self.name, self.pid, ' '.join(self.args))
            logger.error(output)


@singleton
class ProcessMonitor(object):

    def __init__(self):
        self.procs = []
        self.dead_cnt = 0
        self.done = False
        self.is_shutdown = False

    def register(self, p):
        """
        Register process with L{ProcessMonitor}
        @param p: Process
        @type  p: L{Process}
        """
        if self.has_process(p.name):
            logger.error(
                'Cannot add process due to duplicate name "%s".', p.name)
        elif self.is_shutdown:
            logger.error(
                'Cannot add process [%s] due to monitor has been stopped.', p.name)
        else:
            self.procs.append(p)

    def has_process(self, name):
        """
        @return: True if process is still be monitored. If False, process
        has died or was never registered with process
        @rtype: bool
        """
        return len([p for p in self.procs if p.name == name]) > 0

    def check_cleanup(self):
        """
        Check processes are alived, cleanup processes
        """
        dead_cnt = 0
        for pw in self.procs:
            if self.is_shutdown:
                break
            if pw.is_alive():
                continue
            if pw.exception_handler == "respawn":
                if pw.respawn_cnt < pw.respawn_limit:
                    logger.warning(
                        'child process [%s][%d] exit, respawn!', pw.name, pw.pid)
                    pw.start()
                    pw.respawn_cnt += 1
                else:
                    dead_cnt += 1
            elif pw.exception_handler == "exit":
                logger.warning(
                    'child process [%s][%d] exit, stop all', pw.name, pw.pid)
                # stop and exit
                stop()
            else:
                dead_cnt += 1
        if dead_cnt > 0:
            self.dead_cnt = dead_cnt
            if self.dead_cnt == len(self.procs):
                self.is_shutdown = True

    def run(self):
        """
        Run processes monitor, until all processes are died.
        """
        while not self.is_shutdown:
            self.check_cleanup()
            time.sleep(0.2)
        for p in self.procs:
            p.get_exit_state()
        if self.dead_cnt == len(self.procs):
            logger.info("All processes has died.")
            return True
        return False

    def stop(self, signal):
        """
        Stop all processes in monitor
        """
        for p in self.procs:
            if p.is_alive():
                p.popen.send_signal(signal)

        for p in self.procs:
            if p.is_alive():
                logger.warning('Waiting for [%s][%s] exit.', p.name, p.pid)
                try:
                    p.wait(force_stop_timeout_secs)
                except subprocess.TimeoutExpired as e:
                    logger.warning(
                        "Begin force kill process [%s][%s].", p.name, p.pid)
                    p.kill()
                p.wait()
                logger.info(
                    'Process [%s] has been stopped. dag_file: %s', p.name, p.dag_list)
        # Reset members
        self.procs = []
        self.dead_cnt = 0


def get_param_value(module, key, default_value=''):
    """
    Get param value by key from xml conf reader.
    @param module: xml module section
    @param key: param name
    @param default_value: if not found key, return the default value
    """
    value = module.find(key)
    if value is None or value.text is None:
        return default_value
    return value.text.strip()


def get_param_list(module, key):
    """
    Get param list info by key from xml conf reader.
    @param module: xml module section
    @param key: param name
    """
    l = []
    for v in module.findall(key):
        if v.text is None:
            continue
        value = v.text.strip()
        if value:
            l.append(value)
    return l


def start(launch_file=''):
    """
    Start all modules in xml config
    """
    pmon = ProcessMonitor()
    # Find launch file
    if not os.path.isfile(launch_file):
        filename = os.path.join(launch_path, launch_file)
        if os.path.isfile(filename):
            launch_file = filename
        else:
            logger.error('Cannot find launch file: %s ', launch_file)
            sys.exit(1)
    logger.info('Launch file [%s]', launch_file)
    logger.info('=' * 120)

    try:
        tree = ET.parse(launch_file)
    except Exception:
        logger.error('Parse xml failed. illegal xml!')
        sys.exit(1)

    root = tree.getroot()
    # set environment
    for env in root.findall('environment'):
        for var in env.getchildren():
            os.environ[var.tag] = str(var.text)
    # start each process
    for module in root.findall('module'):
        module_name = get_param_value(module, 'name')
        process_name = get_param_value(
            module, 'process_name', 'mainboard_default_' + str(os.getpid()))
        sched_name = get_param_value(module, 'sched_name', 'CYBER_DEFAULT')
        process_type = get_param_value(module, 'type', 'library')
        cpu_profile_file = get_param_value(module, 'cpuprofile')
        mem_profile_file = get_param_value(module, 'memprofile')
        exception_handler = get_param_value(module, 'exception_handler')
        respawn_limit_txt = get_param_value(module, 'respawn_limit')
        if respawn_limit_txt.isnumeric():
            respawn_limit = int(respawn_limit_txt)
        else:
            respawn_limit = g_default_respawn_limit
        nice_val = get_param_value(module, 'nice', 0)
        logger.info('Load module [%s] %s: [%s] [%s] conf, exception_handler: [%s], respawn_limit: [%d]',
                    module_name, process_type, process_name, sched_name, exception_handler, respawn_limit)

        if process_type == 'binary':
            if len(process_name) == 0:
                logger.error(
                    'Start binary failed. Binary process_name is null.')
                continue
            pw = ProcessWrapper(process_name.split()[0], 0, [""], [], process_name, process_type,
                                sched_name, [], exception_handler, respawn_limit, nice=nice_val)
        # Default is library
        else:
            dag_list = get_param_list(module, 'dag_conf')
            if not dag_list:
                logger.error(
                    'module [%s] library dag conf is null.', module_name)
                continue
            plugin_list = get_param_list(module, 'plugin')
            extra_args_list = []
            extra_args = module.attrib.get('extra_args')
            if extra_args is not None:
                extra_args_list = extra_args.split()
            pw = ProcessWrapper(g_binary_name, 0, dag_list, plugin_list,
                                process_name, process_type, sched_name,
                                extra_args_list, exception_handler, respawn_limit,
                                 cpu_profile_file, mem_profile_file, nice_val)
        result = pw.start()
        if result != 0:
            logger.error('Start manager [%s] failed. Stop all!', process_name)
            stop()
        pmon.register(pw)

    # No module in xml
    if not pmon.procs:
        logger.error("No module was found in xml config.")
        return
    all_died = pmon.run()
    if not all_died:
        logger.info("Stop all processes...")
        stop()
    logger.info("Cyber exit.")


def stop(sig=signal.SIGINT):
    """
    stop all modules
    """
    pmon = ProcessMonitor()
    if len(pmon.procs) == 0:
        return
    pmon.stop(sig)

    logger.info('All processes have been stopped.')
    sys.exit(0)


def stop_launch(launch_file):
    """
    Stop the launch file
    """
    if not launch_file:
        cmd = 'pkill -INT cyber_launch'
    else:
        cmd = 'pkill -INT -f ' + launch_file

    os.system(cmd)
    time.sleep(3)
    logger.info('Stop cyber launch finished.')
    sys.exit(0)


def signal_handler(sig, frame):
    logger.info('Keyboard interrupt received. Stop all processes.')
    stop(sig)


def main():
    """
    Main function
    """
    parser = argparse.ArgumentParser(description='cyber launcher')
    subparsers = parser.add_subparsers(help='sub-command help')

    start_parser = subparsers.add_parser(
        'start', help='launch/benchmark.launch')
    start_parser.add_argument('file', nargs='?', action='store',
                              help='launch file, default is cyber.launch')

    stop_parser = subparsers.add_parser(
        'stop', help='stop all the module in launch file')
    stop_parser.add_argument('file', nargs='?', action='store',
                             help='launch file, default stop all the launcher')

    # restart_parser = subparsers.add_parser('restart', help='restart the module')
    # restart_parser.add_argument('file', nargs='?', action='store', help='launch file,
    #                            default is cyber.launch')

    params = parser.parse_args(sys.argv[1:])

    command = sys.argv[1]
    if command == 'start':
        start(params.file)
    elif command == 'stop':
        stop_launch(params.file)
    # elif command == 'restart':
    #    restart(params.file)
    else:
        logger.error('Invalid command %s' % command)
        sys.exit(1)


if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    main()
