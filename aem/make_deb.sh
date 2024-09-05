#!/bin/bash

VERSION=${VERSION:-9.2.0}
PKG_NAME="${PKG_NAME:-apollo-neo-env-manager-dev}"
CODE_NAME="${CODE_NAME:-$(lsb_release -cs)}"
ARCH="${ARCH:-$(dpkg --print-architecture)}"

main() {
  local build_dir="build/${PKG_NAME}_${VERSION}_${ARCH}"
  # create package directory
  rm -rf "${build_dir}"
  mkdir -p "${build_dir}"

  # copy source files
  mkdir -p "${build_dir}/opt/apollo"
  cp -ar aem "${build_dir}/opt/apollo/aem"
  # mkdir -p "${build_dir}/usr/bin"
  # ln -snf ../../../opt/apollo/aem/aem "${build_dir}/usr/bin/aem"

  # create postinst file
  mkdir -p "${build_dir}/DEBIAN"
  cat > "${build_dir}/DEBIAN/postinst" << EOF
#!/bin/sh
ln -snf /opt/apollo/aem/aem /usr/bin/aem
EOF
  chmod +x "${build_dir}/DEBIAN/postinst"

  # create control file
  mkdir -p "${build_dir}/DEBIAN"
  cat > "${build_dir}/DEBIAN/control" << EOF
Package: ${PKG_NAME}
Version: ${VERSION}
Section: Apollo
Priority: optional
Architecture: ${ARCH}
Maintainer: Apollo Developers (apollo-support@baidu.com)
Depends: rsync, tree, lsb-release, python3-dev, python3-pip, python3-venv, python3-jinja2, python3-requests, python3-apt, sudo
Description: Apollo Environment Manager
EOF

  # create deb package
  dpkg-deb --build "${build_dir}"
}

main "$@"
