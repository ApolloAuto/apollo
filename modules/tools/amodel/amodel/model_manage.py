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
import sys
from pathlib import Path

from amodel.model_meta import ModelMeta

# APOLLO_ROOT_DIR
WORKSPACE_PATH = os.getenv('APOLLO_ROOT_DIR', '/apollo')
# Model install paths
MODEL_INSTALL_PATH = "modules/perception/data/models"
# MODEL_META_FILE_NAME
MODEL_META_FILE_NAME = "apollo_deploy"

# Install tmp path
DOWNLOAD_TMP_DIR = "/tmp/"
UNZIP_TMP_DIR = "/tmp/amodel/extract_path"

# Download
DOWNLOAD_REPOSITORY_URL = "https://apollo-pkg-beta.cdn.bcebos.com/perception_model"

# Frame abbreviation
FRAMEWORK_ABBREVIATION = {
  "Caffe": "caffe",
  "PaddlePaddle": "paddle",
  "PyTorch": "torch",
  "TensorFlow": "tf",
  "Onnx": "onnx"}


def _jion_meta_file(meta_path):
  """ Get meta file compatible with different suffixes

  Args:
      meta_path (_type_): meta path

  Returns:
      _type_: meta file
  """
  meta_file = os.path.join(meta_path, "{}.yaml".format(MODEL_META_FILE_NAME))
  if not os.path.isfile(meta_file):
    meta_file = os.path.join(meta_path, "{}.yml".format(MODEL_META_FILE_NAME))
  return meta_file


'''
amodel_list
'''
def _get_model_metas(model_install_path):
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
      meta_file = _jion_meta_file(child_path)
      is_success = model_meta.parse_from(meta_file)
      if is_success:
        model_metas.append(model_meta)
  return model_metas

def _get_all_model_metas():
  """Get all model meta list from model install path(MODEL_INSTALL_PATH)

  Returns:
      list: all installed models's meta
  """
  total_model_metas = []
  absolute_path = os.path.join(WORKSPACE_PATH, MODEL_INSTALL_PATH)
  model_metas = _get_model_metas(absolute_path)
  total_model_metas.extend(model_metas)
  return total_model_metas

def _display_model_list(model_metas, white_names=[]):
  """Display all installed models's info

  Args:
      model_metas (list): all installed models's meta
  """
  # display title
  print("{:<5}|{:<20}|{:<20}|{:<15}|{:<20}|{:<20}".format(
    "ID",
    "Name",
    "Task_type",
    "Sensor_type",
    "Framework",
    "Date"))
  # display content
  for index in range(len(model_metas)):
    model_meta = model_metas[index]
    if len(white_names) == 0 or (model_meta.name in white_names):
      print("{:<5}|{:<20}|{:<20}|{:<15}|{:<20}|{:%Y-%m-%d}".format(
        index,
        model_meta.name,
        model_meta.task_type,
        model_meta.sensor_type,
        model_meta.framework,
        model_meta.date))

def amodel_list():
  """amodel list command
  """
  total_model_metas = _get_all_model_metas()
  _display_model_list(total_model_metas)


'''
amodel_install
'''
def _is_url(url_str):
  """Check url_str is url

  Args:
      url_str (str): url path

  Returns:
      bool: Is url or not
  """
  return url_str.startswith('https://') or url_str.startswith('http://')

def _progress(prefix, cur, total):
  bar_size = 50
  cur_p = int(cur / total * bar_size)
  print("{}[{}{}] {}/{}".format(prefix, "#"*cur_p, "."*(bar_size - cur_p),
      cur, total), end='\r', file=sys.stdout, flush=True)

def _download_from_url(url):
  """Download file from url

  Args:
      url (str): url to download

  Returns:
      file: download file's path
  """
  local_filename = url.split('/')[-1]
  download_file = os.path.join(DOWNLOAD_TMP_DIR, local_filename)

  # File is cached
  if Path(download_file).is_file():
    logging.warn(
      "File downloaded before! use cached file in {}".format(download_file))
    return download_file

  with requests.get(url, stream=True) as r:
    r.raise_for_status()
    chunk_size = 8192
    total_length = int(r.headers.get('content-length', 0)) // chunk_size
    with open(download_file, 'wb') as f:
      for index, chunk in enumerate(r.iter_content(chunk_size)):
        f.write(chunk)
        _progress("Downloading:", index, total_length)
    print()
  return download_file

def _unzip_file(file_path, extract_path):
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

  try:
    shutil.unpack_archive(file_path, extract_path)
  except ValueError:
    logging.error("Unsupported unzip format! {}".format(file_path))
    return False
  except:
    logging.error("Unzip file failed! {}".format(file_path))
    return False
  return True

def _get_install_path_by_meta(model_meta):
  """Get model's install path by meta info

  Args:
      model_meta (object): model's meta

  Returns:
      str: model's install path
  """
  file_name = "{}_{}".format(model_meta.name,
      FRAMEWORK_ABBREVIATION[model_meta.framework])
  install_path = os.path.join(WORKSPACE_PATH, MODEL_INSTALL_PATH, file_name)
  return install_path

def _install_model(model_meta, extract_path, skip_install):
  """Move model from extract_path to install_path

  Args:
      model_meta (object): model's meta
      extract_path (str): model's extract path

  Returns:
      bool: success or not
  """
  install_path = _get_install_path_by_meta(model_meta)
  logging.debug("_install_model: {} -> {}".format(extract_path, install_path))

  # Model exists
  if os.path.isdir(install_path):
    if skip_install:
      logging.warn(
        "Skipped install, model already exist in {}.".format(install_path))
      return True
    confirm = _user_confirmation(
      "Model already exists!!! Do you want to override {}? [y/n]:".format(
        model_meta.name))
    if confirm:
      shutil.rmtree(install_path)
    else:
      return False

  shutil.move(extract_path, install_path)
  print("Installed model in {}.".format(install_path))
  return True

def _construct_default_url(model_path):
  return "{}/{}.zip".format(DOWNLOAD_REPOSITORY_URL, model_path)

def amodel_install(model_path, skip_install = False):
  """amodel install command

  Args:
      model_path (str): model's zip file
  """
  if not model_path:
    logging.error("Input file is empty!!!")
    return

  # If file does not exist and file is not url, use default url
  if not os.path.isfile(model_path) and not _is_url(model_path):
    model_path = _construct_default_url(model_path)

  # Download file
  if _is_url(model_path):
    try:
      model_path = _download_from_url(model_path)
    except Exception as e:
      logging.error("Download {} failed! {}".format(model_path, e))
      return

  # Unzip model file
  _, tail = os.path.split(model_path)
  model_name = tail.split('.')[0]
  is_success = _unzip_file(model_path, UNZIP_TMP_DIR)
  if not is_success:
    logging.error("Model file {} not found.".format(model_path))
    return

  # Read meta file
  model_meta = ModelMeta()
  meta_path = os.path.join(UNZIP_TMP_DIR, model_name)
  meta_file = _jion_meta_file(meta_path)
  is_success = model_meta.parse_from(meta_file)
  if not is_success:
    logging.error("Meta file {} not found!".format(meta_file))
    return

  # Install meta file
  extract_path = os.path.join(UNZIP_TMP_DIR, model_name)
  is_success = _install_model(model_meta, extract_path, skip_install)
  if not is_success:
    logging.error("Failed install {}.".format(model_meta.name))


'''
amodel_remove
'''
def _remove_model_from_path(model_meta):
  """Remove model

  Args:
      model_meta (object): model's meta

  Returns:
      bool: success or not
  """
  install_path = _get_install_path_by_meta(model_meta)
  logging.debug(install_path)
  if os.path.isdir(install_path):
    shutil.rmtree(install_path)
    print("Deleted model in {}.".format(install_path))
    return True
  return False

def _user_confirmation(question):
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
  # Find model index
  if model_name is None:
    logging.error("Input model name or number is empty!!!")
    return

  total_model_metas = _get_all_model_metas()

  model_id = None
  if not model_name.isdigit():
    found_model_names = []
    found_id = 0
    for cur_id in range(len(total_model_metas)):
      model_meta = total_model_metas[cur_id]
      if model_meta.name == model_name:
        found_model_names.append(model_meta)
        found_id = cur_id
    if len(found_model_names) > 1:
      _display_model_list(total_model_metas, found_model_names)
      input_model_id = input("Which model you want to delete, pls input the model ID!")
      if not input_model_id.isdigit():
        logging.error("Please input correct model ID: {}.".format(input_model_id))
        return
      model_id = int(input_model_id)
    elif len(found_model_names) == 1:
      model_id = found_id
    else:
      logging.error("Not found {}, Please check if the name is correct.".format(model_name))
      return

  # Find model_meta by index
  if model_id == None:
    model_id = int(model_name)

  if model_id < len(total_model_metas):
    model_meta = total_model_metas[model_id]
  else:
    logging.error("Not found {}, Please check if the input is correct.".format(model_name))
    return

  model_name = model_meta.name
  # Remove model by model_meta
  confirm = _user_confirmation(
      "Do you want to remove model ID:{} Name:{}? [y/n]:".format(model_id, model_name))
  if confirm:
    is_success = _remove_model_from_path(model_meta)
    if is_success:
      print("Successed remove model ID:{} Name:{}.".format(model_id, model_name))
    else:
      logging.error("Failed remove model ID:{} Name:{}.".format(model_id, model_name))
  else:
    logging.warn("Canceled remove model ID:{} Name:{}.".format(model_id, model_name))


'''
amodel_info
'''
def _display_model_info(model_meta):
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
  # Find model index
  if model_name is None:
    logging.error("Input model name or number is empty!!!")
    return

  total_model_metas = _get_all_model_metas()

  model_id = None
  if not model_name.isdigit():
    found_model_names = []
    found_id = 0
    for cur_id in range(len(total_model_metas)):
      model_meta = total_model_metas[cur_id]
      if model_meta.name == model_name:
        found_model_names.append(model_meta)
        found_id = cur_id
    if len(found_model_names) > 1:
      _display_model_list(total_model_metas, found_model_names)
      input_model_id = input("Which model you want to display, pls input the model ID!")
      if not input_model_id.isdigit():
        logging.error("Please input correct model ID: {}.".format(input_model_id))
        return
      model_id = int(input_model_id)
    elif len(found_model_names) == 1:
      model_id = found_id
    else:
      logging.error("Not found {}, Please check if the name is correct.".format(model_name))
      return

  # Find model_meta by index
  if model_id == None:
    model_id = int(model_name)

  if model_id < len(total_model_metas):
    model_meta = total_model_metas[model_id]
    _display_model_info(model_meta)
  else:
    logging.error("Not found {}, Please check if the input is correct.".format(model_name))
    return
