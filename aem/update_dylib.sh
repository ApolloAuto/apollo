#/bin/bash

APOLLO_LIB_PATH="${APOLLO_ENV_ROOT}/opt/apollo/neo/lib"
LD_CACHE="${APOLLO_ENV_ROOT}/opt/apollo/neo/ld.cache"

readdir() {
  for file in $(ls -r $1); do
    if [ -d $1/$file ]; then
      echo "$1/$file" >> ${APOLLO_ENV_ROOT}/etc/ld.so.conf.d/apollo.conf
      pushd $1/$file > /dev/null
      readdir $1"/"$file
      popd > /dev/null
    fi
  done
}

if [ ! -e ${LD_CACHE} ]; then
  touch ${LD_CACHE}
  chmod a+w ${LD_CACHE}
fi

hash_val=$(tree ${APOLLO_LIB_PATH} | sha256sum | awk '{print $1}')
if [ ! "${hash_val}" = "$(cat ${LD_CACHE})" ]; then
  mkdir -p ${APOLLO_ENV_ROOT}/etc/ld.so.conf.d
  echo "${hash_val}" > ${LD_CACHE}

  touch ${APOLLO_ENV_ROOT}/etc/ld.so.conf.d/apollo.conf
  chmod a+w ${APOLLO_ENV_ROOT}/etc/ld.so.conf.d/apollo.conf
  echo "" > ${APOLLO_ENV_ROOT}/etc/ld.so.conf.d/apollo.conf
  readdir $APOLLO_LIB_PATH
fi

ldconfig -C ${APOLLO_ENV_ROOT}/etc/ld.so.cache 2>&1 > /dev/null
