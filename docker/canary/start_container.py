#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
"""Start apollo docker container."""

import argparse
import os
import platform
import shutil
import subprocess
import sys

APOLLO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))


def shell_cmd(cmd, alert_on_failure=True):
    """Execute shell command and return (ret-code, stdout, stderr)."""
    print('SHELL > {}'.format(cmd))
    proc = subprocess.Popen(cmd, shell=True, close_fds=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    ret = proc.wait()
    stdout = proc.stdout.read() if proc.stdout else None
    stderr = proc.stderr.read() if proc.stderr else None
    if alert_on_failure and stderr and ret != 0:
        sys.stderr.write('{}\n'.format(stderr))
    return (ret, stdout, stderr)


class ArgManager(object):
    """Arguments manager."""

    def __init__(self):
        self.parser = argparse.ArgumentParser(
            description="Start apollo docker container.")
        self.parser.add_argument('--repo', default='apolloauto/apollo',
                                 help='Docker repo name.')
        self.parser.add_argument('--tag', default='dev-x86_64-20180502_canary',
                                 help='Docker image tag.')
        self.parser.add_argument('-C', '--china_mirror', default=False,
                                 action="store_true",
                                 help='Whether to use china mirror.')
        self.parser.add_argument('-l', '--local_image', default=False,
                                 action="store_true",
                                 help='Whether to use local image.')
        self.parser.add_argument('--map', action='append',
                                 help='Optional maps. Make sure it is '
                                 'available as docker volume image at '
                                 '<REPO>:map_volume-<MAP>-latest.')
        self.parser.add_argument('--without_gpu', default=False,
                                 action="store_true",
                                 help='Whether to use GPU. Note that without '
                                 'GPU, it might be not efficient enough for '
                                 'road testing.')

        self._args = None

    def args(self):
        """Get parsed args."""
        if self._args is None:
           self._args = self.parser.parse_args()
        return self._args


class DockerHelper(object):
    """Docker helper."""

    def __init__(self, args):
        self.args = args

    def pull(self, image):
        """Equals `docker pull <image>`."""
        cmd = 'docker pull {}{}'.format(
            'registry.docker-cn.com/' if self.args.china_mirror else '',
            image)
        ret, _, _ = shell_cmd(cmd)
        return (ret == 0)

    def run(self, container, image, options='', entrypoint=''):
        """Equals `docker run -it -d --name <container> <image>`"""
        # Stop existing container.
        cmd = 'docker rm -f "{}"'.format(container)
        shell_cmd(cmd, False)

        cmd = 'docker run -it -d {} --name "{}" "{}" {}'.format(
            options, container, image, entrypoint)
        ret, _, _ = shell_cmd(cmd)
        return (ret == 0)


class DockerContainer(object):
    """Setup a docker container for development."""
    def __init__(self, args):
        self.args = args

        # Docker volumes to be passed as '--volumes-from KEY' to container,
        # where KEY is a container brought up with the image defined by VALUE.
        self.docker_volumes = {
            'apollo_localization_volume':
                '{}:localization_volume-x86_64-latest'.format(args.repo),
            'apollo_yolo3d_volume':
                '{}:yolo3d_volume-x86_64-latest'.format(args.repo),
        }
        # Required and optional map volumes.
        self.add_map_volume(args.repo, 'sunnyvale_big_loop')
        self.add_map_volume(args.repo, 'sunnyvale_loop')
        if args.map:
            for optional_map in args.map:
                self.add_map_volume(args.repo, optional_map)

        # Local volumes to be passed as '-v KEY:VALUE' to container.
        self.local_volumes = {
            APOLLO_ROOT: '/apollo',
        }
        # For systems like 'Darwin', aka, MacOS, skip mounting.
        if platform.system() == 'Linux':
            self.local_volumes.update({
                '/dev': '/dev',
                '/media': '/media',
                '/tmp/.X11-unix': '/tmp/.X11-unix:rw',
                '/etc/localtime': '/etc/localtime:ro',
                '/usr/src': '/usr/src',
                '/lib/modules': '/lib/modules',
            })

        # Environment variables to be passed as '-e KEY=VALUE' to container.
        self.env_vars = {
            'DISPLAY': os.environ['DISPLAY'] or ':0',
            'DOCKER_USER_ID': os.getuid(),
            'DOCKER_GRP_ID': os.getgid(),
            'DOCKER_IMG': '{}:{}'.format(self.args.repo, self.args.tag),
        }

        self.other_options = [
            '--privileged',
            '--net host',
            '--add-host in_dev_docker:127.0.0.1',
            '--hostname in_dev_docker',
            '--shm-size 2G',
        ]
        if not args.without_gpu:
            self.other_options.append('--runtime nvidia')

    def add_map_volume(self, repo, map_name, version='latest'):
        """Add map volume to apollo container."""
        container_name = 'apollo_map_{}'.format(map_name)
        image_name = '{}:map_volume-{}-{}'.format(repo, map_name, version)
        self.docker_volumes[container_name] = image_name

    def check_env(self):
        """Check if the environment is valid."""
        # Check user.
        if os.geteuid() == 0:
            sys.stderr.write(
                'Please run Apollo as general user instead of root!\n')
            return False
        # Check agreement.
        return self.check_agreement()

    def check_agreement(self):
        """Check if the user has agreed to the Apollo license agreement."""
        agreed_agreement = os.path.expanduser('~/.apollo_agreement.txt')
        if os.path.exists(agreed_agreement):
            return True

        source_agreement = os.path.join(APOLLO_ROOT, 'scripts/AGREEMENT.txt')
        with open(source_agreement, 'r') as fin:
            print(fin.read())
            print('========\nType "y" to agree to the license agreement above, '
                  'or any other key to exit.')
            if sys.stdin.read(1) == 'y':
                shutil.copyfile(source_agreement, agreed_agreement)
                return True
        return False

    def run(self):
        """Run container."""
        if not self.check_env():
            return False

        docker_helper = DockerHelper(self.args)

        # If --local_image is not set, try pulling all required images first.
        if not self.args.local_image:
            if not docker_helper.pull(self.env_vars['DOCKER_IMG']):
                return False
            for volume_image in self.docker_volumes.values():
                if not docker_helper.pull(volume_image):
                    return False

        # Start docker volumes.
        for container, image in self.docker_volumes.items():
            if not docker_helper.run(container, image):
                return False

        # Construct options and start container.
        docker_volumes_option = ' '.join(
            ['--volumes-from "{}"'.format(key)
                for key in self.docker_volumes.keys()])
        local_volumes_option = ' '.join(
            ['-v "{}":"{}"'.format(key, val)
                for key, val in self.local_volumes.items()])
        env_vars_option = ' '.join(
            ['-e "{}"="{}"'.format(key, val)
                for key, val in self.env_vars.items()])
        options = ' '.join([
            docker_volumes_option,
            local_volumes_option,
            env_vars_option,
            ' '.join(self.other_options),
        ])

        success = docker_helper.run(
            'apollo_dev', self.env_vars['DOCKER_IMG'], options,
            '/apollo/docker/canary/setup_container.sh')
        if success:
            print('The container has been brought up, now you can run:')
            print('  {}/docker/canary/dev_into.sh'.format(APOLLO_ROOT))
            print('to play with it. Enjoy!')
        return success


def main():
    """Main entry."""
    arg_manager = ArgManager()
    container = DockerContainer(arg_manager.args())
    container.run()

if __name__ == '__main__':
    main()
