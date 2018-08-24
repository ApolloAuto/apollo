#! /bin/bash
if [ -z $1 ]; then
    echo "[Usage] build_cov.sh bazel_target(modules/...)"
    exit 0
fi
cd /apollo
target=$1
#target=$(realpath $1)
#target=$(echo ${target:6})
echo $target
bazel test --crosstool_top=tools/clang-6.0:toolchain $target -c dbg --config=coverage

