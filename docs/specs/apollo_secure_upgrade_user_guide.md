# Apollo Secure Upgrade SDK

Currently, most software upgrade solutions are not securely protected.
Therefore, devices are exposed to various security threats during the upgrade
procedure. Apollo secure upgrade SDK provides secure upgrade capabilities, and
can be easily integrated to make the upgrade process more secure and robust.

## Features

1. Packages are encrypted and signature protected in storing and transmitting
   phases.
2. Server and device can authenticate each other.
3. Cryptographic resources are protected properly.
4. Server provides customized authorizations to different devices.
5. Prevent attackers utilizing the server’s response to replay attack devices.
6. Multiple platforms (Ubuntu 14, Centos 6, Centos 7 and Andorid) are supported.

## Upgrade Procedure

A typical upgrade procedure is shown below:

![](images/apollo_unsecure_upgrade_procedure.jpg)

1. The upgrade server generates the upgrade package.
2. Packages are uploaded to the storage server.
3. Storage server sends the package URL to upgrade server.
4. Device sends the upgrade request to upgrade server.
5. Upgrade server replies package URL to the device.
6. Device requests package from the storage server.
7. Packages are downloaded to device.
8. Device installs the package.

After integrating secure upgrade SDK, the upgrade procedure is modified as
follows:

![](images/apollo_secure_upgrade_procedure.jpg)

1. The upgrade server generates the secure package and package token.
2. Secure packages and the package token are uploaded to the storage server.
3. The storage server sends secure package and package token URLs to the upgrade
   server.
4. The device generates the device token and sends to the upgrade server with
   the upgrade request.
5. The upgrade server generates authorization token and sends the token to the
   device with a replied secure package URL.
6. Device requests secure package from the storage server.
7. Secure packages are downloaded to device.
8. Device verifies the secure package with the authorization token, and
   generates the original package.
9. Device installs the package.

## User Guide

### 1. SDK Layout

SDK contains four directories:

1. python API: python interface.
2. config: SDK root configuration file, log file.
3. certificate: certificate file.
4. depend_lib: dependency libraries.

### 2. Interfaces

#### a) Initialize

This function should be called before using secure upgrade APIs.

```
init_secure_upgrade(root_config_path)
input para:
  root_config_path  root configuration file path
```

#### b) Device Token Generation

This function is used to generate the device token.

```
sec_upgrade_get_device_token()
Output para:
  return code: true    generating device token successfully
               false    generating device token failed
  Device_token: device token (string format)
```

#### c) Package Generation

This function is used to generate the secure upgrade package and package token.

```
sec_upgrade_get_package(original_package_path,
                        secure_package_path,
                        package_token_path)
input para:
    original_package_path    original upgrade package file path
    secure_package_path    secure upgrade package file path
    package_token_path    secure package token file
output para:
    return code:
        true    generating secure upgrade package successfully
        false    generating secure upgrade package failed
```

#### d) Authorization Token Generation

This function is used to generate a device’s authorization token, based on
device token and package token.

```
sec_upgrade_get_authorization_token(package_token_path,
                                    device_token_path)
input para:
    package_token_path    secure package token file path
	device_token_path    device token file path
output_para:
    return code:
        true    generating authorization token successfully
        false    generating authorization token failed
        authorization_token authorization token buffer(string formate)
```

#### e) Authorization Token and Package Verification

This function is used to verify the downloaded secure package with the
authorization token and generate the original package.

```
sec_upgrade_verify_package(authorization_token_buffer,
                           secure_package_path)
input para:
    authorization_token_buffer    authorization token buffer(string format)
    secure_package_path    secure upgrade package file path
output para:
    original_package_path    original upgrade package file path
```

### 3. Additional Information

1. SDK uses standard PEM certificates.
2. Before using SDK, users need to generate two seperate chains of certificate
   for server and device.
3. Certificates from the server certificate chain are deployed to server and
   make sure they cannot sign other certificates.
4. Certificates from the device certificate chain are deployed to device and
   make sure they cannot sign other certificates.
5. Root private key should not be deployed to server or devices.
6. Users need to be assigned the read and write permissions of the `config`
   directory and the read permission of the `certificate` directory.
