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
import yaml
import argparse
import os
import sys
import logging
import glob
import time
script_path = os.path.dirname(os.path.realpath(__file__))

def record_check(args):
    state = args.state
    record_path = args.record_path
    import channel_check as m_channel_check
    ChannelChecker = m_channel_check.ChannelChecker
    channel_chekcer = ChannelChecker(os.path.join(script_path, "client.yaml"))
    ret = 0
    if state == "start":
        ret = channel_chekcer.async_start(record_path)
    else:
        ret = channel_chekcer.async_stop()
    if ret != 0:
        logging.error("async_start failed, record_path [%s]" % record_path)
        return -1
    logging.info("async_start succeed, you can move on to the next step")
    return 0

def static_align(args):
    state = args.state
    import static_align as m_static_align
    StaticAlign = m_static_align.StaticAlign
    static_align = StaticAlign(os.path.join(script_path, "client.yaml"))
    ret = static_align.sync_start()
    if ret != 0:
        logging.info("static align failed")
        return -1
    return 0

def eight_route(args):
    state = args.state
    import eight_route as m_eight_route
    EightRoute = m_eight_route.EightRoute
    eight_route = EightRoute(os.path.join(script_path, "client.yaml"))
    ret = eight_route.sync_start()
    if ret != 0:
        logging.info("eight_route failed")
        return -1
    return 0

def data_collect(args):
    state = args.state
    conf = None
    with open(os.path.join(script_path, 'client.yaml'), 'r') as fp:
        conf = yaml.load(fp.read())
    time_file = os.path.join(script_path, conf['data_collect']['time_flag_file'])
    if not os.path.exists(time_file):
        open(time_file, 'w').close()
    with open(time_file, 'w') as fp:
        lines = fp.readlines()
        if state == "start":
            if len(lines) == 0:
                fp.write("%s start\n" % str(time.time()))
            else:
                the_last_line = lines[-1]
                s = the_last_line.strip().split()
                if s[0] == "start":
                    logging.info("already start, this command will be ignored")
                else:
                    fp.write("%s start\n" % str(time.time()))
        else:
            if len(lines) == 0:
                logging.info("start first, this command will be ignored")
            else:
                the_last_line = lines[-1]
                s = the_last_line.strip().split()
                if s[0] == "start":
                    fp.write("%s end\n" % str(time.time()))
                else:
                    logging.info("already end, this command will be ignored")

        

def loops_check(args):
    state = args.state
    import loops_check as m_loops_check
    LoopsChecker = m_loops_check.LoopsChecker
    loops_checker = LoopsChecker(os.path.join(script_path, "client.yaml"))
    ret = loops_checker.sync_start()
    if ret != 0:
        logging.info("loops_check failed")
        return -1
    return 0

def state_machine(args):
    conf = None
    with open(os.path.join(script_path, 'client.yaml'), 'r') as fp:
        conf = yaml.load(fp.read())
    if os.path.exists(os.path.join(script_path, conf['channel_check']['stop_flag_file'])):
        os.remove(os.path.join(script_path, conf['channel_check']['stop_flag_file']))
    if os.path.exists(os.path.join(script_path, conf['data_collect']['time_flag_file'])):
        os.remove(os.path.join(script_path, conf['data_collect']['time_flag_file']))
    logging.info("state machine clean done")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(help='sub-command help')

    parser_record_check = subparsers.add_parser('record_check', help='record check help')
    parser_record_check.add_argument('--state', required=True)
    parser_record_check.add_argument('--record_path')
    parser_record_check.set_defaults(func=record_check)

    parser_static_align = subparsers.add_parser('static_align', help='static_align help')
    parser_static_align.add_argument('--state', required=True)
    parser_static_align.set_defaults(func=static_align)

    parser_eight_route = subparsers.add_parser('eight_route', help='eight_route help')
    parser_eight_route.add_argument('--state', required=True)
    parser_eight_route.set_defaults(func=eight_route)

    parser_loops_check = subparsers.add_parser('data_collect', help='data_collect help')
    parser_loops_check.add_argument('--state', required=True)
    parser_loops_check.set_defaults(func=data_collect)

    parser_loops_check = subparsers.add_parser('loops_check', help='loops_check help')
    parser_loops_check.add_argument('--state', required=True)
    parser_loops_check.set_defaults(func=eight_route)

    parser_state_machine = subparsers.add_parser('state_machine', help='state mechine')
    parser_state_machine.add_argument('--action', required=True)
    parser_state_machine.set_defaults(func=state_machine)

    args = parser.parse_args()
    ret = args.func(args)
    print glob.glob(os.path.join(script_path, "client.py"))
    if ret != 0:
        logging.error("run %s failed!" % (' '.join(sys.argv)))
        sys.exit(-1)
    
    sys.exit(0)
