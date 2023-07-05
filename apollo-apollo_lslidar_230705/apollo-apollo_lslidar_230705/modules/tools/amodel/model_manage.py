#!/usr/bin/env python3

###############################################################################
# Copyright 2022 The Apollo Authors. All Rights Reserved.
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
amodel command implement
"""

import logging
import os
import requests
import shutil
import zipfile

from model_meta import ModelMeta

# APOLLO_ROOT_DIR
WORKSPACE_PATH = os.getenv('APOLLO_ROOT_DIR', '/apollo')
# MODEL_META_FILE_NAME
MODEL_META_FILE_NAME = "apollo_deploy.yaml"

# Install tmp path
DOWNLOAD_TMP_DIR = "/tmp/"
UNZIP_TMP_DIR = "/tmp/amodel/extract_path"

# Model install paths
MODEL_INSTALL_PATH = {
  "3d_detection_lidar": "modules/perception/production/data/perception/lidar/models/detection",
  "3d_segmentation_lidar": "modules/perception/production/data/perception/lidar/models/cnnseg",
  "3d_detection_camera": "modules/perception/production/data/perception/camera/models/yolo_obstacle_detector",
  "lane_detection_camera": "modules/perception/production/data/perception/camera/models/lane_detector",
  "tl_detection_camera": "modules/perception/production/data/perception/camera/models/traffic_light_detection",
  "tl_recognition_camera": "modules/perception/production/data/perception/camera/models/traffic_light_recognition",
}

# Frame abbreviation
FRAMEWORK_ABBREVIATION = {
  "Caffe": "caffe",
  "PaddlePaddle": "paddle",
  "PyTorch": "torch",
  "TensorFlow": "tf"}


'''
amodel_list
'''
def get_model_metas(model_install_path):
  """Get model metas from path

  Args:
      model_install_path (str): file path

  Returns:
      list: model meta list
  """
  model_metas = []
  if not os.path.isdir(model_install_path):
    return model_metas

  # Find MODEL_META_FILE_NAME in child directories.
  for model_path in os.listdir(model_install_path):
    child_path = os.path.join(model_install_path, model_path)
    if os.path.isdir(child_path):
      model_meta = ModelMeta()
      meta_file = os.path.join(child_path, MODEL_META_FILE_NAME)
      is_success = model_meta.parse_from(meta_file)
      if is_success:
        model_metas.append(model_meta)
  return model_metas

def get_all_model_metas():
  """Get all model meta list from model install path(MODEL_INSTALL_PATH)

  Returns:
      list: all installed models's meta
  """
  total_model_metas = []
  for _, model_install_path in MODEL_INSTALL_PATH.items():
    absolute_path = os.path.join(WORKSPACE_PATH, model_install_path)
    model_metas = get_model_metas(absolute_path)
    total_model_metas.extend(model_metas)
  return total_model_metas

def display_model_list(model_metas):
  """Display all installed models's info

  Args:
      model_metas (list): all installed models's meta
  """
  # display title
  print("{:<20}|{:<20}|{:<20}|{:<20}|{:<20}".format(
    "Name",
    "Task_type",
    "Sensor_type",
    "Framework",
    "Date"))
  # display content
  for model_meta in model_metas:
    print("{:<20}|{:<20}|{:<20}|{:<20}|{:%Y-%m-%d}".format(
      model_meta.name,
      model_meta.task_type,
      model_meta.sensor_type,
      model_meta.framework,
      model_meta.date))

def amodel_list():
  """amodel list command
  """
  total_model_metas = get_all_model_metas()
  display_model_list(total_model_metas)


'''
amodel_install
'''
def is_url(url_str):
  """Check url_str is url

  Args:
      url_str (str): url path

  Returns:
      bool: Is url or not
  """
  return url_str.startswith('https://') or url_str.startswith('http://')

def download_from_url(url):
  """Download file from url

  Args:
      url (str): url to download

  Returns:
      file: download file's path
  """
  local_filename = url.split('/')[-1]
  download_file = os.path.join(DOWNLOAD_TMP_DIR, local_filename)
  with requests.get(url, stream=True) as r:
    r.raise_for_status()
    with open(download_file, 'wb') as f:
      for chunk in r.iter_content(chunk_size=8192):
        f.write(chunk)
  return download_file

def unzip_file(file_path, extract_path):
  """Unzip file_path to extract_path

  Args:
      file_path (str): zip file need to unzip
      extract_path (str): unzip path

  Returns:
      bool: success or not
  """
  if not os.path.isfile(file_path):
    return False
  if os.path.isdir(extract_path):
    shutil.rmtree(extract_path)

  with zipfile.ZipFile(file_path, 'r') as zip_ref:
    zip_ref.extractall(extract_path)
  return True

def get_install_path_by_meta(model_meta):
  """Get model's install path by meta info

  Args:
      model_meta (object): model's meta

  Returns:
      str: model's install path
  """
  perception_task = "{}_{}".format(model_meta.task_type, model_meta.sensor_type)
  file_name = "{}_{}".format(model_meta.name,
      FRAMEWORK_ABBREVIATION[model_meta.framework])
  install_path = os.path.join(
      WORKSPACE_PATH,
      MODEL_INSTALL_PATH[perception_task],
      file_name)
  return install_path

def install_model(model_meta, extract_path):
  """Move model from extract_path to install_path

  Args:
      model_meta (object): model's meta
      extract_path (str): model's extract path

  Returns:
      bool: success or not
  """
  install_path = get_install_path_by_meta(model_meta)
  logging.debug("install_model: {} -> {}".format(extract_path, install_path))

  if os.path.isdir(install_path):
    confirm = user_confirmation(
      "Model already exists!!! Do you want to override {}? [y/n]:".format(
        model_meta.name))
    if confirm:
      shutil.rmtree(install_path)
    else:
      return False

  shutil.move(extract_path, install_path)
  return True

def amodel_install(model_path):
  """amodel install command

  Args:
      model_path (str): model's zip file
  """
  if not model_path:
    print("Input file is empty!!!")
    return

  # download file
  if is_url(model_path):
    try:
      model_path = download_from_url(model_path)
    except Exception as e:
      print("Download {} failed! {}".format(model_path, e))
      return

  # unzip model file
  _, tail = os.path.split(model_path)
  model_name = tail.split('.')[0]
  is_success = unzip_file(model_path, UNZIP_TMP_DIR)
  if not is_success:
    print("Zip file {} not found.".format(model_path))
    return

  # read meta file
  model_meta = ModelMeta()
  meta_file = os.path.join(UNZIP_TMP_DIR, model_name, MODEL_META_FILE_NAME)
  is_success = model_meta.parse_from(meta_file)
  if not is_success:
    print("Meta file {} not found!".format(meta_file))
    return

  # install meta file
  extract_path = os.path.join(UNZIP_TMP_DIR, model_name)
  is_success = install_model(model_meta, extract_path)
  if is_success:
    print("Successed install {}.".format(model_meta.name))
  else:
    print("Failed install {}.".format(model_meta.name))


'''
amodel_remove
'''
def remove_model_from_path(model_meta):
  """Remove model

  Args:
      model_meta (object): model's meta

  Returns:
      bool: success or not
  """
  install_path = get_install_path_by_meta(model_meta)
  logging.debug(install_path)
  if os.path.isdir(install_path):
    shutil.rmtree(install_path)
    return True
  return False

def user_confirmation(question):
  """Command line confirmation interaction

  Args:
      question (str): Prompt user for confirmation

  Returns:
      bool: sure or not
  """
  yes_choices = ['yes', 'y']
  no_choices = ['no', 'n']
  count = 3
  while count > 0:
    count -= 1
    user_input = input(question)
    if user_input.lower() in yes_choices:
      return True
    elif user_input.lower() in no_choices:
      return False
  return False

def amodel_remove(model_name):
  """amodel remove command

  Args:
      model_name (str): the model need to remove
  """
  total_model_metas = get_all_model_metas()
  for model_meta in total_model_metas:
    if model_meta.name == model_name:
      confirm = user_confirmation(
          "Do you want to remove {}? [y/n]:".format(model_name))
      if confirm:
        is_success = remove_model_from_path(model_meta)
        if is_success:
          print("Successed remove {}.".format(model_name))
        else:
          print("Failed remove {}.".format(model_name))
      else:
        print("Canceled remove {}.".format(model_name))
      return
  # Not found
  print("Not found {}, Please check if the name is correct.".format(model_name))


'''
amodel_info
'''
def display_model_info(model_meta):
  """Display model info

  Args:
      model_meta (object): model's meta
  """
  print(model_meta)

def amodel_info(model_name):
  """amodel info command

  Args:
      model_name (str): model's name
  """
  total_model_metas = get_all_model_metas()
  for model_meta in total_model_metas:
    if model_meta.name == model_name:
      display_model_info(model_meta)
      return
  # Not found
  print("Not found {}, Please check if the name is correct.".format(model_name))
