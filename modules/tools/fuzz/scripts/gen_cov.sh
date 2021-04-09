cd /apollo/
COV_DIR=data/cov
  rm -rf $COV_DIR
  files=$(find bazel-out/local_clang-dbg/bin/modules/ -iname "*.gcda" -o -iname "*.gcno" | grep -v external)
for f in $files; do
    target="$COV_DIR/objs/modules/${f##*modules}"
    mkdir -p "$(dirname "$target")"
    cp "$f" "$target"
  done

files=$(find bazel-out/local-opt/bin/modules/ -iname "*.gcda" -o -iname "*.gcno" | grep -v external)
for f in $files; do
  target="$COV_DIR/objs/modules/${f##*modules}"
  mkdir -p "$(dirname "$target")"
  cp "$f" "$target"
done

lcov --rc lcov_branch_coverage=1 --capture --directory "$COV_DIR/objs" --output-file "$COV_DIR/conv.info"
if [ $? -ne 0 ]; then
  echo 'lcov failed!'
  exit $?
fi
lcov --rc lcov_branch_coverage=1 --remove "$COV_DIR/conv.info" \
    "external/*" \
    "/usr/*" \
    "bazel-out/*" \
-o $COV_DIR/stripped_conv.info
genhtml $COV_DIR/stripped_conv.info --output-directory $COV_DIR/report
echo "Generated coverage report in $COV_DIR/report/index.html"
