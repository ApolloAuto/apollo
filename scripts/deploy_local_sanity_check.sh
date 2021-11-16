#!/bin/bash
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

##Install
#$APOLLO_ROOT/scripts/deploy_local_sanity_check.sh

##Uninstall
#$APOLLO_ROOT/scripts/deploy_local_sanity_check.sh -u

APOLLO_ROOT=$(cd $(dirname $0)/.. && pwd)
APOLLO_FILE="modules/dreamview"

. $APOLLO_ROOT/scripts/apollo_base.sh

if [ ! -e "$APOLLO_FILE" ]; then
  warning "Please run this script under Apollo source root dir."
  exit 1
fi

if [ ! -e "$APOLLO_ROOT/.git" ]; then
  warning "$APOLLO_ROOT seems not a git repo."
  exit 1
fi

type curl > /dev/null 2>&1 || {
  error >&2 "command curl not found, please install it with: sudo apt-get install curl"
  exit 1
}
type perl > /dev/null 2>&1 || {
  error >&2 "command perl not found, please install it with: sudo apt-get install perl-base"
  exit 1
}

function uninstall() {
  if [ -L "$APOLLO_ROOT/.git/hooks/post-commit" ]; then
    pushd $APOLLO_ROOT/.git/hooks > /dev/null
    rm post-commit
    popd > /dev/null
    ok "sanity check was removed."
  fi
}

while [ $# -gt 0 ]; do
  case "$1" in
    -u) uninstall && exit 0 ;;
    *) ;;
  esac
  shift
done

HOOKS_URL="http://code.qt.io/cgit/qt/qtrepotools.git/plain/git-hooks"
HOOKS_DIR=$APOLLO_ROOT/tools/git-hooks
HOOK_SCRITPS="git_post_commit_hook sanitize-commit"

if [ ! -e $HOOKS_DIR ]; then
  mkdir -p $HOOKS_DIR
fi

pushd $HOOKS_DIR > /dev/null || error "Enter $HOOKS_DIR failed."

for i in $HOOK_SCRITPS; do
  if [ ! -e "$HOOKS_DIR/$i" ]; then
    #info "pulling hooks: $i ..."
    curl -O $HOOKS_URL/$i
    if [ $? -ne 0 ]; then
      error "Failed to pull hooks: $i ."
    fi
    chmod +x $i
  fi
done

popd > /dev/null

if [ ! -e "$APOLLO_ROOT/.git/hooks/post-commit" ]; then
  pushd $APOLLO_ROOT/.git/hooks > /dev/null || error "Enter target dir failed. "
  #info "deploy hooks..."
  ln -s $HOOKS_DIR/git_post_commit_hook post-commit
  if [ $? -eq 0 ]; then
    ok "Deploy sanity check done."
  else
    error "Failed to deploy sanity check."
  fi
  popd > /dev/null
elif [ -L "$APOLLO_ROOT/.git/hooks/post-commit" ]; then
  info "Sanity check seems already deployed."
elif [ -f "$APOLLO_ROOT/.git/hooks/post-commit" ]; then
  info "$APOLLO_ROOT/.git/hooks/post-commit hook seems already exists, please backup it and run this script again."
fi
