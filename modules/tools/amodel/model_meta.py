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
amodel meta implement
"""

import logging
import os
import yaml


class ModelMeta(object):
  """Model meta object

  Args:
      object (_type_): base object
  """
  def __init__(self, name=None, date=None, task_type=None, sensor_type=None,
      framework=None, model=None, dataset=None) -> None:
    self.name        = name
    self.date        = date
    self.task_type   = task_type
    self.sensor_type = sensor_type
    self.framework   = framework
    self.model       = model
    self.dataset     = dataset

    # raw yaml data
    self._raw_modle_meta = None

  def parse_from(self, meta_file_path):
    """Parse model meta from yaml file

    Args:
        meta_file_path (str): Meta yaml file

    Returns:
        bool: Success or fail
    """
    if not os.path.isfile(meta_file_path):
      return False
    with open(meta_file_path, 'r') as meta_fp:
      self._raw_modle_meta = yaml.safe_load(meta_fp)
      logging.debug(self._raw_modle_meta)
      self.name        = self._raw_modle_meta["name"]
      self.date        = self._raw_modle_meta["date"]
      self.task_type   = self._raw_modle_meta["task_type"]
      self.sensor_type = self._raw_modle_meta["sensor_type"]
      self.framework   = self._raw_modle_meta["framework"]
      self.model       = self._raw_modle_meta["model"]
      self.dataset     = self._raw_modle_meta["dataset"]
    return True

  def save_to(self, meta_file_path):
    """Save model meta to yaml file

    Args:
        meta_file_path (str): Meta yaml file
    """
    with open(meta_file_path, 'w') as meta_fp:
      yaml.safe_dump(self._raw_modle_meta, meta_fp, sort_keys=False)

  def __str__(self) -> str:
    """Model meta string

    Returns:
        str: Model meta content
    """
    return yaml.safe_dump(self._raw_modle_meta, sort_keys=False)
