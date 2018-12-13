#!/usr/bin/env python

import secure_upgrade_export as sec_api
import os
import sys
sys.path.append('/home/caros/secure_upgrade/python')

root_config_path = "/home/caros/secure_upgrade/config/secure_config.json"
returnCode = sec_api.init_secure_upgrade(root_config_path)
if returnCode == True:
    print 'Security environment init successfully!'
else:
    print 'Security environment init fail!'
    exit()

homedir = os.environ['HOME']
release_tgz = homedir + '/.cache/apollo_release.tar.gz'
sec_release_tgz = homedir + '/.cache/sec_apollo_release.tar.gz'
package_token = homedir + '/.cache/package_token'

returnCode = sec_api.sec_upgrade_get_package(
    release_tgz, sec_release_tgz, package_token)
if returnCode == True:
    print 'Security package generated successfully!'
else:
    print 'Security package generated fail!'
