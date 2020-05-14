ulimit -c unlimited

. /apollo/scripts/apollo_base.sh

alias bb="bazel build --distdir=/apollo/.cache/distdir"
alias bt="bazel test  --distdir=/apollo/.cache/distdir"

function inc() {
    /bin/grep "#include" -r $1 | awk -F':' '{print $2}' | sort -u
}

