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
import traceback

import xml.etree.ElementTree as ET


g_binary_name = 'mainboard'
g_pwd = os.getcwd()
g_script_name = os.path.basename(sys.argv[0]).split(".")[0]
g_process_pid = os.getpid()
g_process_name = g_script_name + "_" + str(g_process_pid)

cyber_path = os.getenv('CYBER_PATH')

"""
colorful logging
"""
BLACK, RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, WHITE = list(range(8))
RESET_SEQ = "\033[0m"
COLOR_SEQ = "\033[1;%dm"
BOLD_SEQ = "\033[1m"

COLORS = {
    'INFO':     GREEN,
    'WARNING':  YELLOW,
    'DEBUG':    BLUE,
    'ERROR':    RED,
    'CRITICAL': YELLOW
}


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
logger = logging.Logger(__name__)
logger.addHandler(console)


def exit_handler():
    stop()
    os.chdir(g_pwd)
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
            logger.debug('%s# %s' % (mod.name, line.decode('utf8').strip('\n')))
            continue
        time.sleep(0.01)


class ProcessWrapper(object):

    def __init__(self, binary_path, dag_num, dag_list, process_name,
                 process_type, sched_name, exception_handler=''):
        self.time_of_death = None
        self.started = False
        self.binary_path = binary_path
        self.dag_num = dag_num
        self.dag_list = dag_list
        self.name = process_name
        self.sched_name = sched_name
        self.process_type = process_type
        self.popen = None
        self.exit_code = None
        self.args = []
        self.pid = -1
        self.exception_handler = exception_handler

    def wait(self):
        if self.started:
            self.popen.wait()

    def start(self):
        """
        Start a manager in process name
        """
        if self.process_type == 'binary':
            args_list = self.name.split()
        else:
            args_list = [self.binary_path, '-d'] + self.dag_list
            if len(self.name) != 0:
                args_list.append('-p')
                args_list.append(self.name)
            if len(self.sched_name) != 0:
                args_list.append('-s')
                args_list.append(self.sched_name)

        self.args = args_list

        try:
            self.popen = subprocess.Popen(args_list, stdout=subprocess.PIPE,
                                          stderr=subprocess.STDOUT)
        except Exception as err:
            logger.error('Subprocess Popen exception: ' + str(err))
            return 2
        else:
            if self.popen.pid == 0 or self.popen.returncode is not None:
                logger.error('Start process [%s] failed.' % self.name)
                return 2

        th = threading.Thread(target=module_monitor, args=(self, ))
        th.setDaemon(True)
        th.start()
        self.started = True
        self.pid = self.popen.pid
        logger.info('Start process [%s] successfully. pid: %d' %
                    (self.name, self.popen.pid))
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
                'Cannot add process due to duplicate name "%s".' % p.name)
        elif self.is_shutdown:
            logger.error(
                'Cannot add process [%s] due to monitor has been stopped.' % p.name)
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
            if pw.process_type == 'binary':
                continue
            try:
                if not pw.is_alive():
                    if pw.exception_handler == "respawn":
                        logger.warn(
                            'child process [%s][%d] exit, respawn!' % (pw.name, pw.pid))
                        result = pw.start()
                        if result != 0:
                            logger.error(
                                'respawn process [%s] failed, stop all!' % (pw.name))
                            stop()
                    elif pw.exception_handler == "exit":
                        logger.warn(
                            'child process [%s][%d] exit, stop all' % (pw.name, pw.pid))
                        stop()
                    dead_cnt += 1
            except Exception:
                dead_cnt += 1
                traceback.print_exc()
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
                logger.warn('Waiting for [%s][%s] exit.' % (p.name, p.pid))
                p.wait()
                logger.info(
                    'Process [%s] has been stopped. dag_file: %s' % (p.name, p.dag_list))
        # Reset members
        self.procs = []
        self.dead_cnt = 0


def start(launch_file=''):
    """
    Start all modules in xml config
    """
    pmon = ProcessMonitor()
    # Find launch file
    if launch_file[0] == '/':
        launch_file = launch_file
    elif launch_file == os.path.basename(launch_file):
        launch_file = os.path.join(cyber_path, 'launch', launch_file)
    else:
        if os.path.exists(os.path.join(g_pwd, launch_file)):
            launch_file = os.path.join(g_pwd, launch_file)
        else:
            logger.error('Cannot find launch file: %s ' % launch_file)
            sys.exit(1)
    logger.info('Launch file [%s]' % launch_file)
    logger.info('=' * 120)

    if not os.path.isfile(launch_file):
        logger.error('Launch xml file %s does not exist' % launch_file)
        sys.exit(1)

    try:
        tree = ET.parse(launch_file)
    except Exception:
        logger.error('Parse xml failed. illegal xml!')
        sys.exit(1)
    total_dag_num = 0
    dictionary = {}
    dag_dict = {}
    root1 = tree.getroot()
    for module in root1.findall('module'):
        dag_conf = module.find('dag_conf').text
        process_name = module.find('process_name').text
        process_type = module.find('type')
        if process_type is None:
            process_type = 'library'
        else:
            process_type = process_type.text
            if process_type is None:
                process_type = 'library'
            process_type = process_type.strip()
        if process_type != 'binary':
            if dag_conf is None or not dag_conf.strip():
                logger.error('Library dag conf is null')
                continue
            if process_name is None:
                process_name = 'mainboard_default_' + str(os.getpid())
            process_name = process_name.strip()
            if str(process_name) in dictionary:
                dictionary[str(process_name)] += 1
            else:
                dictionary[str(process_name)] = 1
            if str(process_name) not in dag_dict:
                dag_dict[str(process_name)] = [str(dag_conf)]
            else:
                dag_dict[str(process_name)].append(str(dag_conf))
            if dag_conf is not None:
                total_dag_num += 1

    process_list = []
    root = tree.getroot()
    for env in root.findall('environment'):
        for var in env.getchildren():
            os.environ[var.tag] = str(var.text)
    for module in root.findall('module'):
        module_name = module.find('name').text
        dag_conf = module.find('dag_conf').text
        process_name = module.find('process_name').text
        sched_name = module.find('sched_name')
        process_type = module.find('type')
        exception_handler = module.find('exception_handler')
        if process_type is None:
            process_type = 'library'
        else:
            process_type = process_type.text
            if process_type is None:
                process_type = 'library'
            process_type = process_type.strip()

        if sched_name is None:
            sched_name = "CYBER_DEFAULT"
        else:
            sched_name = sched_name.text

        if process_name is None:
            process_name = 'mainboard_default_' + str(os.getpid())
        if dag_conf is None:
            dag_conf = ''
        if module_name is None:
            module_name = ''
        if exception_handler is None:
            exception_handler = ''
        else:
            exception_handler = exception_handler.text
        module_name = module_name.strip()
        dag_conf = dag_conf.strip()
        process_name = process_name.strip()
        sched_name = sched_name.strip()
        exception_handler = exception_handler.strip()

        logger.info('Load module [%s] %s: [%s] [%s] conf: [%s] exception_handler: [%s]' %
                    (module_name, process_type, process_name, sched_name, dag_conf,
                     exception_handler))

        if process_name not in process_list:
            if process_type == 'binary':
                if len(process_name) == 0:
                    logger.error(
                        'Start binary failed. Binary process_name is null.')
                    continue
                pw = ProcessWrapper(
                    process_name.split()[0], 0, [
                        ""], process_name, process_type,
                    exception_handler)
            # Default is library
            else:
                pw = ProcessWrapper(
                    g_binary_name, 0, dag_dict[
                        str(process_name)], process_name,
                    process_type, sched_name, exception_handler)
            result = pw.start()
            if result != 0:
                logger.error(
                    'Start manager [%s] failed. Stop all!' % process_name)
                stop()
            pmon.register(pw)
            process_list.append(process_name)

    # No module in xml
    if not process_list:
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
    if cyber_path is None:
        logger.error(
            'Error: environment variable CYBER_PATH not found, set environment first.')
        sys.exit(1)
    os.chdir(cyber_path)
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
    signal.signal(signal.SIGCHLD, signal.SIG_IGN)
    main()
