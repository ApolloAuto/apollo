#!/usr/bin/env python

###############################################################################
# Copyright 2019 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

"""
This is a tool to extract useful information from given record files. It does
self-check the validity of the uploaded data and able to inform devloper's when
the data is not qualified, and reduce the size of uploaded data significantly.

TODO:
* Validate the data file, refer to cyber_info especially about the header?
* Develop algorithm to extract useful information (JPG, PCD, Binary data)
* Cloud support
* Use GZIP format by default, maybe we should consider XZ to archieve high
  compression ratio. However, LZMA has problem of compatibility between
  Python2.x and Python3.x.
"""

import sys
import os
import gzip
#import shutil
import argparse

CYBER_PATH = os.getenv('CYBER_PATH')

#TODO: Add bindary metadata
EXTRACTION_INFO = ['JPG', 'PCD', ]

def find_extraction_info(line):
    """
    Examine if the input line includes usefull data, return true if found
    """
    pass


def extract_data(record_path, output_path):
    """
    Extract the desired information from record file
    @record_path: the given record file
    @output_path: compressed file for output, gzip by default
    """
    with open(record_path, 'rb') as f_in, gzip.open(output_path + 'gz', 'wb') as f_out:
        for line in f_in:
            result = find_extraction_info(line)
            if result is True:
                f_out.write(line)


#def compress_file(extracted_file, output_file, compress_type='gzip'):
#    """
#    Compress the extracted file to GZIP package
#    """
#    if compress_type == 'gzip':
#        with open(extracted_file, 'rb') as f_in, gzip.open(output_file, 'wb') as f_out:
#            shutil.copyfileobj(f_in, f_out)


def validate_data(record_path):
    """
    Check the validity of record file
    """
    pass


def main():
    """
    Main function
    """
    if CYBER_PATH is None:
        print('Error: environment variable CYBER_PATH not found, set environment first.')
        sys.exit(1)
    os.chdir(CYBER_PATH)

    parser = argparse.ArgumentParser(description='Calibration data extraction tool')
    parser.add_argument("--record_path", action="store", type=str, help="the input record file")
    parser.add_argument("--output_path", action="store", type=str, default="./extracted.bin",
                        help="The output compressed file")
    args = parser.parse_args()

    record_path = args.record_path
    result = validate_data(args.record_path)
    if result is False:
        sys.exit(1)

    extract_data(record_path, args.output_path)


if __name__ == '__main__':
    main()
