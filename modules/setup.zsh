#!/usr/bin/env zsh
# generated from catkin/cmake/templates/setup.zsh.in

CATKIN_SHELL=zsh

# source setup.sh from same directory as this file
_CATKIN_SETUP_DIR=$(builtin cd -q "`dirname "$0"`" > /dev/null && pwd)
emulate -R zsh -c 'source "$_CATKIN_SETUP_DIR/setup.sh"'
