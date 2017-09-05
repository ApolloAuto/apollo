#!/usr/bin/env bash

Domain=apollo.car
IP=127.0.0.1
KEYS_DIR="$(dirname $0)"/../modules/common/data/ssl_keys

# Clear KEYS_DIR and cd into it.
rm -fr "${KEYS_DIR}"
mkdir -p "${KEYS_DIR}"
cd "${KEYS_DIR}"

function GenerateCAKeys() {
  echo "Generate CA keys..."
  openssl genpkey -algorithm RSA -pkeyopt rsa_keygen_bits:2048 -out ca.key
  openssl req -new -x509 -sha256 -days 3650 -key ca.key \
      -subj "/O=Apollo/CN=apollo.ca" -out ca.crt
}

function GenerateServerKeys() {
  echo "Generate server keys..."
  openssl genpkey -algorithm RSA -pkeyopt rsa_keygen_bits:2048 -out server.key
  openssl req -new -sha256 -key server.key \
      -subj "/O=Apollo/OU=HMI/CN=${Domain}" -out server.csr

  OPENSSL_CONF_FILE="./openssl.conf"
  echo "[req]req_extensions = v3_req
        [v3_req]
        subjectAltName = @alt_names
        [alt_names]
        IP.1 = ${IP}
        DNS.1 = ${Domain}" > ${OPENSSL_CONF_FILE}
  openssl x509 -sha256 -days 3650 -req \
      -extfile ${OPENSSL_CONF_FILE} -extensions v3_req \
      -in server.csr -CAcreateserial -CA ca.crt -CAkey ca.key -out server.crt

  openssl verify -verbose -CAfile ca.crt server.crt

  cat server.crt >> server.pem
  cat server.key >> server.pem
  sed -i -e 's/BEGIN PRIVATE KEY/BEGIN RSA PRIVATE KEY/g' server.pem
  sed -i -e 's/END PRIVATE KEY/END RSA PRIVATE KEY/g' server.pem
}

function PrepareHost() {
  sudo cp ca.crt /usr/local/share/ca-certificates/apollo_ca.crt
  sudo update-ca-certificates

  HOST_CONF="${IP} ${Domain}"
  if ! grep -Fxq "$HOST_CONF" /etc/hosts; then
    echo ${HOST_CONF} | sudo tee -a /etc/hosts
  fi
}

GenerateCAKeys
GenerateServerKeys
PrepareHost

echo "Generated keys:"
ls -al
