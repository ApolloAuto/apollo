#!/usr/bin/env python
import yaml
import sys
import os

f = open(sys.argv[1])
transform_stamped = yaml.safe_load(f)
# manual remap novatel and velodyne64 for motion compensatro
if len(sys.argv) >= 3 and sys.argv[2] == "remap_novatel" \
    and transform_stamped['header']['frame_id'] == "novatel" \
    and transform_stamped['child_frame_id'] == "velodyne64":
    transform_stamped['header']['frame_id'] = "perception_localization_100hz"
    transform_stamped['child_frame_id'] = "velodyne64_localization"
command = 'static_transform_broadcaster '\
        '%f %f %f %f %f %f %f %s %s' % (transform_stamped['transform']['translation']['x'],\
        transform_stamped['transform']['translation']['y'],\
        transform_stamped['transform']['translation']['z'],\
        transform_stamped['transform']['rotation']['x'],\
        transform_stamped['transform']['rotation']['y'],\
        transform_stamped['transform']['rotation']['z'],\
        transform_stamped['transform']['rotation']['w'],\
        transform_stamped['header']['frame_id'],\
        transform_stamped['child_frame_id'])

print command
ret = os.system(command)
print ret
