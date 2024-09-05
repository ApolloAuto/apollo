[api]
meta_api=https://apollo.baidu.com/packages/api/depo
download_query=https://apollo.baidu.com/packages/api/package/url
download=https://apollo.baidu.com/packages/api/package/download
login=https://apollo.baidu.com/packages/api/login
attr_query=https://apollo.baidu.com/packages/api/package/attr
register_api = https://apollo.baidu.com/packages/api/register
version_available_api = https://apollo.baidu.com/packages/api/tools_version
apollo_open_maps_api_entrypoint = https://apollo.baidu.com/open-maps/api/v1

[cache]
offline_metadata_prefix=${APOLLO_ENV_ROOT}/opt/apollo/neo/share/buildtool/namespaces
installed_package=${APOLLO_ENV_ROOT}/opt/apollo/neo/share/buildtool/installed_package
user_installed_package=$buildtool/install/user
mock_install_target_file=buildtool/mock/BUILD
offline_packages_filename=Packages
ld_cache=ld.cache
offline_cyberfile_cache_filename=Cyberfiles
id_token=${APOLLO_ENV_ROOT}/opt/apollo/neo/share/buildtool/id_token
id_token_tmp=/tmp/id_token
apollo_maps_download_path=/tmp

[decider]
targets=${APOLLO_ENV_ROOT}/opt/apollo/neo/share/buildtool/targets
results=${APOLLO_ENV_ROOT}/opt/apollo/neo/share/buildtool/results
cyberfile_source=${APOLLO_ENV_ROOT}/opt/apollo/neo/share/buildtool/cyberfile_source

[base]
external_path=/usr/include
apollo_root=${APOLLO_ENV_ROOT}/opt/apollo/neo
apollo_package_path=${APOLLO_ENV_ROOT}/opt/apollo/neo/packages
deprecated_package_path=packages
apollo_lib_path=${APOLLO_ENV_ROOT}/opt/apollo/neo/lib
apollo_include_path=${APOLLO_ENV_ROOT}/opt/apollo/neo/include
apollo_maps_path=${APOLLO_ENV_ROOT}/apollo/modules/map/data
apollo_maps_install_path=${APOLLO_ENV_ROOT}/apollo/modules/map/data

source_path_prefix=src
binary_path_prefix=bin
include_path_prefix=include
python_path_prefix=python
library_path_prefix=lib
package_meta_prefix=share/packages
config_path_prefix=share
playgroud_prefix=playgroud

[compile]
march=-march=native

[setting]
host=apollo.baidu.com
version=9.0.0-rc1-r19
request_timeout=5

