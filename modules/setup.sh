#!/usr/bin/env sh
# generated from catkin/cmake/template/setup.sh.in

# Sets various environment variables and sources additional environment hooks.
# It tries it's best to undo changes from a previously sourced setup file before.
# Supported command line options:
# --extend: skips the undoing of changes from a previously sourced setup file
#   (in plain sh shell which does't support arguments for sourced scripts you
#   can set the environment variable `CATKIN_SETUP_UTIL_ARGS=--extend` instead)

# since this file is sourced either use the provided _CATKIN_SETUP_DIR
# or fall back to the destination set at configure time
: ${_CATKIN_SETUP_DIR:=/home/tongsky/Documents/Github/apollo/modules}
_SETUP_UTIL="$_CATKIN_SETUP_DIR/_setup_util.py"
unset _CATKIN_SETUP_DIR

if [ ! -f "$_SETUP_UTIL" ]; then
  echo "Missing Python script: $_SETUP_UTIL"
  return 22
fi

# detect if running on Darwin platform
_UNAME=`uname -s`
_IS_DARWIN=0
if [ "$_UNAME" = "Darwin" ]; then
  _IS_DARWIN=1
fi
unset _UNAME

# make sure to export all environment variables
export CMAKE_PREFIX_PATH
if [ $_IS_DARWIN -eq 0 ]; then
  export LD_LIBRARY_PATH
else
  export DYLD_LIBRARY_PATH
fi
unset _IS_DARWIN
export PATH
export PKG_CONFIG_PATH
export PYTHONPATH

# remember type of shell if not already set
if [ -z "$CATKIN_SHELL" ]; then
  CATKIN_SHELL=sh
fi

# invoke Python script to generate necessary exports of environment variables
# use TMPDIR if it exists, otherwise fall back to /tmp
if [ -d "${TMPDIR}" ]; then
  _TMPDIR="${TMPDIR}"
else
  _TMPDIR=/tmp
fi
_SETUP_TMP=`mktemp "${_TMPDIR}/setup.sh.XXXXXXXXXX"`
unset _TMPDIR
if [ $? -ne 0 -o ! -f "$_SETUP_TMP" ]; then
  echo "Could not create temporary file: $_SETUP_TMP"
  return 1
fi
CATKIN_SHELL=$CATKIN_SHELL "$_SETUP_UTIL" $@ $CATKIN_SETUP_UTIL_ARGS >> "$_SETUP_TMP"
_RC=$?
if [ $_RC -ne 0 ]; then
  if [ $_RC -eq 2 ]; then
    echo "Could not write the output of '$_SETUP_UTIL' to temporary file '$_SETUP_TMP': may be the disk if full?"
  else
    echo "Failed to run '\"$_SETUP_UTIL\" $@': return code $_RC"
  fi
  unset _RC
  unset _SETUP_UTIL
  rm -f "$_SETUP_TMP"
  unset _SETUP_TMP
  return 1
fi
unset _RC
unset _SETUP_UTIL
. "$_SETUP_TMP"
rm -f "$_SETUP_TMP"
unset _SETUP_TMP

# source all environment hooks
_i=0
while [ $_i -lt $_CATKIN_ENVIRONMENT_HOOKS_COUNT ]; do
  eval _envfile=\$_CATKIN_ENVIRONMENT_HOOKS_$_i
  unset _CATKIN_ENVIRONMENT_HOOKS_$_i
  eval _envfile_workspace=\$_CATKIN_ENVIRONMENT_HOOKS_${_i}_WORKSPACE
  unset _CATKIN_ENVIRONMENT_HOOKS_${_i}_WORKSPACE
  # set workspace for environment hook
  CATKIN_ENV_HOOK_WORKSPACE=$_envfile_workspace
  . "$_envfile"
  unset CATKIN_ENV_HOOK_WORKSPACE
  _i=$((_i + 1))
done
unset _i

unset _CATKIN_ENVIRONMENT_HOOKS_COUNT
