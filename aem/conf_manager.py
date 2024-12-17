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
Data Cleaner: auto clean record and log
"""

import argparse
import os
import re
import signal
import sys
import logging

# logging
FORMAT = '[%(levelname)s] %(asctime)s [line:%(lineno)s] %(message)s'
logging.basicConfig(format=FORMAT, level=logging.DEBUG)
logger = logging.getLogger('apollo')

APOLLO_CONF_ROOT = os.getenv("APOLLO_RUNTIME_PATH", '/apollo')


def quit(signum, _):
    """ quit
    """
    print('quit by signum: %s' % signum)
    sys.exit()


def update(source_dir, target_dir, is_force=False):
    """ udpate conf
    Params:
        - source_dir: the source dir path
        - target_dir: the target dir path
        - is_force: is force overwrrite or not
    """
    for root, _, files in os.walk(source_dir):
        for name in files:
            filename = os.path.join(root, name)
            relative_dir_path = root[len(source_dir):].lstrip('/')
            target_dir_path = os.path.join(target_dir, relative_dir_path)
            target_filename = os.path.join(target_dir_path, name)
            if os.path.isdir(target_filename):
                logger.error("%s is a dir, please remove it manually.",
                             target_filename)
            if not is_force and os.path.isfile(target_filename):
                # ignored if not force and a valid file or link exists.
                continue
            # ignored if is same link
            if os.path.islink(target_filename) and os.readlink(
                    target_filename) == filename:
                continue
            # make sure target dir exists
            if os.path.isfile(target_dir_path):
                logger.error("%s is a file, please remove it manually.",
                             target_dir_path)
                return False
            os.makedirs(target_dir_path, exist_ok=True)
            # remove previous file
            if os.path.exists(target_filename) or os.path.islink(
                    target_filename):
                os.remove(target_filename)
            # create symbolic link
            os.symlink(filename, target_filename)
    return True


def recover(target_dir, recovered_dir):
    """ recover target dir by fix invalid link
    Params:
        - target_dir: the dir which to be recovered
        - is_recover: is recover invalid path
        - recovered_dir: the recovered dir path
    """
    succ = True
    for root, _, files in os.walk(target_dir):
        for name in files:
            filename = os.path.join(root, name)
            if not os.path.islink(filename):
                continue
            if os.path.exists(filename):
                continue
            recovered_filename = os.path.join(
                recovered_dir, filename[len(target_dir):].lstrip('/'))
            logger.info("begin recover file: %s", filename)
            if not os.path.isfile(recovered_filename):
                logger.warn("recover %s error, recovered file not exists.",
                            filename)
                succ = False
                continue
            os.remove(filename)
            os.symlink(recovered_filename, filename)
    return succ


def _update(args):
    """ update func
    """
    for d in [args.source_dir, args.target_dir]:
        if not os.path.isdir(d):
            logger.error("dir [%s] not exists!", d)
            sys.exit(1)
    if args.sub_dirs:
        for d in args.sub_dirs:
            if not os.path.isdir(os.path.join(args.source_dir, d)):
                logger.error("sub dir [%s] not exists!", d)
                sys.exit(1)
    logger.info("begin update conf...")
    try:
        if args.sub_dirs:
            for d in args.sub_dirs:
                source_dir = os.path.abspath(os.path.join(args.source_dir, d))
                target_dir = os.path.abspath(os.path.join(args.target_dir, d))
                if not update(source_dir, target_dir, args.force):
                    logger.error("update conf failed!")
                    sys.exit(1)
        elif not update(os.path.abspath(args.source_dir),
                        os.path.abspath(args.target_dir), args.force):
            logger.error("update conf failed!")
            sys.exit(1)
    except Exception as e:
        logger.error("update conf error: %s", e)
        sys.exit(1)
    logger.info("finish update conf.")


def _recover(args):
    """ recover func
    """
    if not args.recovered_dir:
        if not os.getenv("APOLLO_ROOT_DIR"):
            logger.error('APOLLO_ROOT_DIR not found!')
            sys.exit(1)
        #args.recovered_dir = os.path.join(os.environ['APOLLO_ROOT_DIR'], 'src')
        args.recovered_dir = os.path.join(os.environ['APOLLO_ROOT_DIR'])
    if not os.path.isdir(args.recovered_dir):
        logger.error("recovered source dir [%s] not exists!",
                     args.recovered_dir)
        sys.exit(1)

    logger.info("begin recover conf...")
    try:
        if not recover(os.path.abspath(args.target_dir),
                       os.path.abspath(args.recovered_dir)):
            logger.error("recover conf failed!")
            sys.exit(1)
    except Exception as e:
        logger.error("recover conf error: %s", e)
        sys.exit(1)

    logger.info("finish recover conf.")


def main():
    """ Main Method
    """
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)

    parser = argparse.ArgumentParser(description='Apollo Conf manager')
    subparsers = parser.add_subparsers(help='sub-command help', dest="cmd")

    update_parser = subparsers.add_parser('update', help='update command help')
    update_parser.add_argument('-s',
                               '--source_dir',
                               help='source dir path',
                               required=True)
    update_parser.add_argument('-sub',
                               '--sub_dirs',
                               help='sub dir that need to be updated',
                               nargs='+',
                               required=False),
    update_parser.add_argument('-t',
                               '--target_dir',
                               help='target dir path',
                               default=APOLLO_CONF_ROOT)
    update_parser.add_argument('-f',
                               '--force',
                               help='force overwrite',
                               action='store_true')
    update_parser.set_defaults(func=_update)

    recover_parser = subparsers.add_parser('recover',
                                           help='recover command help')
    recover_parser.add_argument('-t',
                                '--target_dir',
                                help='the dir which to be recovered',
                                default=APOLLO_CONF_ROOT)
    recover_parser.add_argument('-r',
                                '--recovered_dir',
                                help='recovered source dir path',
                                required=False)
    recover_parser.set_defaults(func=_recover)
    args = parser.parse_args()
    if args.cmd:
        args.func(args)
    else:
        parser.print_help()


if __name__ == '__main__':
    main()
