SRC_DIR=./proto
DST_DIR=./proto
protoc -I=$SRC_DIR --python_out=$DST_DIR $SRC_DIR/*.proto
