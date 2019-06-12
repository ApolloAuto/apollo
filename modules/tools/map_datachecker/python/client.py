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

"""map_datachecker client"""
from __future__ import print_function
import yaml
import argparse
import os
import sys
import logging
import glob
import time
script_path = os.path.dirname(os.path.realpath(__file__))
conf_path = os.path.join(os.path.dirname(script_path), "conf/client.yaml")
logging.basicConfig(level = logging.INFO, format = '[%(asctime)s %(filename)s:%(lineno)s] %(message)s', stream = sys.stdout)
def record_check(args):
    command = args.command
    record_path = args.record_path
    import channel_check as m_channel_check
    ChannelChecker = m_channel_check.ChannelChecker
    channel_chekcer = ChannelChecker(conf_path)
    ret = 0
    logging.info("command: " + command)
    if command == "start":
        if not record_path:
            logging.error("Parameter record_path is required when command is start")
            print("Parameter record_path is required when command is start", file=sys.stderr)
            return -1
        record_path = record_path.strip()
        if not os.path.exists(record_path):
            logging.error("Parameter record_path [%s] error" % record_path)
            print("Parameter record_path [%s] error" % record_path, file=sys.stderr)
            return -1
        ret = channel_chekcer.async_start(record_path)
        if ret != 0:
            logging.error("async_start failed, record_path [%s]" % record_path)
            return -1
    else:
        ret = channel_chekcer.async_stop()
        if ret != 0:
            logging.error("async_stop failed, record_path [%s]" % record_path)
            return -1
    logging.info("async operation succeed, you can move on to the next step")
    print("async operation succeed, you can move on to the next step", file=sys.stderr)
    return 0

def static_align(args):
    command = args.command
    import static_align as m_static_align
    StaticAlign = m_static_align.StaticAlign
    static_align = StaticAlign(conf_path)
    ret = 0
    if not command or command == "start":
        ret = static_align.sync_start()
        print("Static aligh succeed", file=sys.stderr)
        print("Next, you may want to run: bash client.sh eight_route start", file=sys.stderr)
    elif command == "stop":
        ret = static_align.sync_stop()
    else:
        logging.error("Error command, expected command are [start|stop], current command is [%s]" % command)
        print("Error command, expected command are [start|stop], current command is [%s]" % command, file=sys.stderr)
        return -1
    if ret != 0:
        logging.info("static align failed")
        return -1
    return 0

def eight_route(args):
    command = args.command
    import eight_route as m_eight_route
    EightRoute = m_eight_route.EightRoute
    eight_route = EightRoute(conf_path)
    ret = 0
    if not command or command == "start":
        ret = eight_route.sync_start()
        print("Eight route succeed", file=sys.stderr)
        print("Next, you may want to run: bash client.sh data_collect start", file=sys.stderr)
    elif command == "stop":
        ret = eight_route.sync_stop()
    else:
        logging.error("Error command, expected command are [start|stop], current command is [%s]" % command)
        print("Error command, expected command are [start|stop], current command is [%s]" % command, file=sys.stderr)
        return -1
    if ret != 0:
        logging.info("eight_route failed")
        return -1
    return 0

def data_collect(args):
    command = args.command
    conf = None
    with open(conf_path, 'r') as fp:
        conf = yaml.load(fp.read())
    time_file = os.path.join(script_path, conf['data_collect']['time_flag_file'])
    if not os.path.exists(time_file):
        open(time_file, 'w').close()
    with open(time_file, 'a+') as fp:
        lines = fp.readlines()
        now = str(time.time())
        if command == "start":
            if len(lines) == 0:
                fp.write("%s start\n" % now)
                logging.info("write [%s start] to file [%s]" % (now, time_file))
                print("Start success. At the end of the collection, you should run: bash client.sh data_collect stop", file=sys.stderr)
            else:
                the_last_line = lines[-1]
                s = the_last_line.strip().split()
                if s[1] == "start":
                    logging.info("This progress has been already started, this command will be ignored")
                    print("This progress has been already started, this command will be ignored", file=sys.stderr)
                else:
                    fp.write("%s start\n" % now)
                    logging.info("Write [%s start] to file [%s]" % (now, time_file))
                    print("Start success. At the end of the collection, you should run: bash client.sh data_collect stop", file=sys.stderr)
        elif command == "stop":
            if len(lines) == 0:
                logging.info("Start first, this command will be ignored")
            else:
                the_last_line = lines[-1]
                s = the_last_line.strip().split()
                if s[1] == "start":
                    fp.write("%s stop\n" % now)
                    logging.info("write [%s stop] to file [%s]" % (now, time_file))
                    print("Stop success. Next you may want to run: bash client.sh loops_check start", file=sys.stderr)
                else:
                    logging.info("This progress has been already stopped, this command will be ignored")
                    print("This progress has been already stopped, this command will be ignored", file=sys.stderr)
        else:
            logging.error("Error command, expected command are [start|stop], current command is [%s]" % command)
            print("Error command, expected command are [start|stop], current command is [%s]" % command, file=sys.stderr)
    return 0

def loops_check(args):
    command = args.command
    import loops_check as m_loops_check
    LoopsChecker = m_loops_check.LoopsChecker
    loops_checker = LoopsChecker(os.path.join(os.path.dirname(script_path), "conf/client.yaml"))
    [ret, res] = loops_checker.sync_start()
    if ret != 0:
        logging.info("loops_check failed")
        return -1
    if res.loop_result.is_reached:
        logging.info("loops meet requirements, res: %s" % str(res))
        print("Loops meet requirements. Next you may want to run: bash client.sh eight_route start", file=sys.stderr)
    else:
        logging.info("loops not meet requirements, res: %s" % str(res))
        print("Loops do not meet requirements, the actual loop number is:%d" % res.loop_result.loop_num, file=sys.stderr)
        print("Next you may need to run: bash client.sh data_collect start", file=sys.stderr)
    return 0

def state_machine(args):
    conf = None
    with open(os.path.join(os.path.dirname(script_path), 'conf/client.yaml'), 'r') as fp:
        conf = yaml.load(fp.read())
    if os.path.exists(os.path.join(script_path, conf['channel_check']['stop_flag_file'])):
        os.remove(os.path.join(script_path, conf['channel_check']['stop_flag_file']))
    if os.path.exists(os.path.join(script_path, conf['data_collect']['time_flag_file'])):
        os.remove(os.path.join(script_path, conf['data_collect']['time_flag_file']))
    logging.info("state machine clean done")
    print("state machine clean done", file=sys.stderr)
    return 0

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='sub-command help')

    parser_record_check = subparsers.add_parser('record_check', help='record check help')
    parser_record_check.add_argument('command')
    parser_record_check.add_argument('--record_path')
    parser_record_check.set_defaults(func=record_check)

    parser_static_align = subparsers.add_parser('static_align', help='static_align help')
    parser_static_align.add_argument('command')
    parser_static_align.set_defaults(func=static_align)

    parser_eight_route = subparsers.add_parser('eight_route', help='eight_route help')
    parser_eight_route.add_argument('command')
    parser_eight_route.set_defaults(func=eight_route)

    parser_loops_check = subparsers.add_parser('data_collect', help='data_collect help')
    parser_loops_check.add_argument('command')
    parser_loops_check.set_defaults(func=data_collect)

    parser_loops_check = subparsers.add_parser('loops_check', help='loops_check help')
    parser_loops_check.add_argument('command')
    parser_loops_check.set_defaults(func=loops_check)

    parser_state_machine = subparsers.add_parser('clean', help='state mechine')
    # parser_state_machine.add_argument('command')
    parser_state_machine.set_defaults(func=state_machine)

    args = parser.parse_args()
    ret = args.func(args)
    # print args
    # print args.func
    # ret = record_check(args)
    # print glob.glob(os.path.join(script_path, "client.py"))
    if ret != 0:
        logging.error("run %s failed!" % (' '.join(sys.argv)))
        sys.exit(-1)
    logging.error("run %s succeed!" % (' '.join(sys.argv)))
    sys.exit(0)
