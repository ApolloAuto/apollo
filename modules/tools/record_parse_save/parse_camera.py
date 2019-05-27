"""
function to parse camera images from *.record files, created using Apollo-Auto

parsed data is saved to *.jpeg file, for each capture

"""

import sys, os
from cyber_py import cyber
from cyber_py import record
from modules.drivers.proto.sensor_image_pb2 import CompressedImage

###########################################################
def parse_data(channelname, msg, out_folder):
    """
    parser images from Apollo record file
    """
    msg_camera = CompressedImage()
    msg_camera.ParseFromString(str(msg))

    tstamp = msg_camera.measurement_time

    temp_time = str(tstamp).split('.')
    if len(temp_time[1])==1:
        temp_time1_adj = temp_time[1] + '0'
    else:
        temp_time1_adj = temp_time[1]
    image_time = temp_time[0] + '_' + temp_time1_adj

    image_filename = "image_" + image_time + ".jpeg"
    f = open(out_folder + image_filename, 'w+b')
    f.write(msg_camera.data)
    f.close()

    return tstamp

###########################################################
