#! /usr/bin/env bash

: ${STAGE_DIR:=/tmp/apollo}
export STAGE_DIR
export APOLLO_DIR=/opt/apollo

function retrieve_so_deps() {
    ldd $1 | awk '/=>/ {print $3}' | sort -u | \
        grep -Ev "^(${APOLLO_DIR}|/usr/local|${STAGE_DIR})/"
}
export -f retrieve_so_deps

function generate_solibs() {
    listing="$1"
    SYSROOT=${APOLLO_DIR}/sysroot
    find ${STAGE_DIR}/bin ${SYSROOT}/bin -executable -type f  \
        -exec bash -c 'retrieve_so_deps "$0"' {}  \
            >> ${listing} \;
    find ${STAGE_DIR}/lib ${STAGE_DIR}/cyber ${STAGE_DIR}/modules \
        -name "*.so" -exec bash -c 'retrieve_so_deps "$0"' {}  \
            >> ${listing} \;

    SYSLIB_DIRS=(
        /usr/local/fast-rtps/lib
        /usr/local/libtorch_cpu/lib
        /usr/local/libtorch_gpu/lib
        ${APOLLO_DIR}/sysroot/lib
    )

    for libdir in ${SYSLIB_DIRS[@]}; do
        find ${libdir} -name "*.so" \
            -exec bash -c 'retrieve_so_deps "$0"' {} \
                >> ${listing} \;
    done
    cat ${listing} | sort -u
}

function solib_locate() {
    solib="$1"
    dest="$2"
    # https://superuser.com/questions/363444
    # /how-do-i-get-the-output-and-exit-value-of-a-subshell-when-using-bash-e
    if ! msg="$(dpkg -S ${solib} 2>/dev/null)" ; then
        echo "Warning: ${solib} doesn't seem to belong to any APT package."
    else
        result="$(echo "${msg}" | awk -F ': ' '{print $1}')"
        echo "${result}" >> ${dest}
    fi
}

PKGS_TXT="${STAGE_DIR}/pkgs.txt"
function generate_apt_pkgs() {
    sudo apt-get -y update
    listing="$(mktemp /tmp/syslibs.XXXXXX)"
    pkgs="$(mktemp /tmp/pkgs.XXXXXX)"
    for solib in $(generate_solibs "${listing}"); do
        solib_locate "${solib}" "${pkgs}"
    done
    sort -u ${pkgs} > ${PKGS_TXT}
    rm -f ${listing} ${pkgs}
}

function main() {
    # TODO(storypku):
    # parse_cmdline_args
    bazel run --config=opt --config=gpu //:install \
        -- --pre_clean \
        "${STAGE_DIR}"

    find "${STAGE_DIR}" -name "*.dag" -exec \
        sed -i 's@/apollo/bazel-bin@/apollo@g' {} \;

    generate_apt_pkgs
}

main "$@"
