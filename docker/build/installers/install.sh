#! /usr/bin/env bash
INSTALLERS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
GEOLOC=cn


function main() {
    sudo cp -r ${INSTALLERS_DIR}/../rcfiles /opt/apollo/

    bash ${INSTALLERS_DIR}/install_geo_adjustment.sh ${GEOLOC}
    bash ${INSTALLERS_DIR}/install_minimal_environment.sh ${GEOLOC}
    bash ${INSTALLERS_DIR}/install_cmake.sh
    bash ${INSTALLERS_DIR}/install_cyber_deps.sh
    bash ${INSTALLERS_DIR}/install_llvm_clang.sh
    bash ${INSTALLERS_DIR}/install_qa_tools.sh
    bash ${INSTALLERS_DIR}/install_visualizer_deps.sh
    bash ${INSTALLERS_DIR}/install_bazel.sh

    bash ${INSTALLERS_DIR}/install_modules_base.sh
    bash ${INSTALLERS_DIR}/install_gpu_support.sh
    bash ${INSTALLERS_DIR}/install_ordinary_modules.sh
    bash ${INSTALLERS_DIR}/install_drivers_deps.sh
    bash ${INSTALLERS_DIR}/install_dreamview_deps.sh ${GEOLOC}
}

main "$@"
