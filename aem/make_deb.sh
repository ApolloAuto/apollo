#!/bin/bash

export SCRIPT_DIR=$(
  cd $(dirname $0)
  pwd
)
source ${SCRIPT_DIR}/env.sh

VERSION=${AEM_VERSION}
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
mkdir -p /etc/bash_completion.d && ln -snf /opt/apollo/aem/auto_complete.bash /etc/bash_completion.d/aem
mkdir -p /usr/share/zsh/functions/Completion/Unix && ln -snf /opt/apollo/aem/auto_complete.zsh /usr/share/zsh/functions/Completion/Unix/_aem
EOF
  chmod +x "${build_dir}/DEBIAN/postinst"

  cat > "${build_dir}/DEBIAN/prerm" << EOF
#!/bin/sh
rm -f /etc/bash_completion.d/aem
rm -f /usr/share/zsh/functions/Completion/Unix/_aem
rm -f /usr/bin/aem
rm -rf /opt/apollo/neo/packages/env-manager-dev
rm -rf /usr/local/bin/aem
EOF
  chmod +x "${build_dir}/DEBIAN/prerm"

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

  # create cyberfile
  cat > "${build_dir}.cyberfile" << EOF
<package format="2">
  <name>env-manager-dev</name>
  <version>${VERSION}</version>
  <description>
    Apollo env manager.
  </description>

  <maintainer email="apollo-support@baidu.com">Apollo Team</maintainer>
  <license>Apache License 2.0</license>
  <url type="website">https://www.apollo.auto/</url>
  <url type="repository">https://github.com/ApolloAuto/apollo</url>
  <url type="bugtracker">https://github.com/ApolloAuto/apollo/issues</url>

  <src_path>//None</src_path>
  <type>pure-binary</type>

</package>
EOF
}

main "$@"
