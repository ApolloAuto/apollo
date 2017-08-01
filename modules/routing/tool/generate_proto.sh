#!/usr/bin/env bash

WORK_ROOT="../../../.."
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROTO_DIR=$CURR_DIR/$WORK_ROOT/baidu/adu-3rd/protobuf
PROTO_LIB=$PROTO_DIR/lib
LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$PROTO_LIB
export LD_LIBRARY_PATH
LIBRARY_PATH=$LIBRARY_PATH:$PROTO_LIB
export LIBRARY_PATH

proto_file_path=$CURR_DIR/../src/proto
proto_file_output=$CURR_DIR/gen
proto_bin=$PROTO_DIR/bin/protoc

if [ -d $proto_file_output ]; then
    rm -r $proto_file_output
fi
mkdir $proto_file_output
cp $proto_file_path/*.proto $proto_file_output
cp $CURR_DIR/$WORK_ROOT/baidu/adu/common/proto/*/*.proto $proto_file_output

touch $proto_file_output/__init__.py

$proto_bin --proto_path=$proto_file_output --python_out=$proto_file_output $proto_file_output/*.proto
