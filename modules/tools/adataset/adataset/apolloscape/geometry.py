#!/usr/bin/env python3

###############################################################################
# Copyright 2023 The Apollo Authors. All Rights Reserved.
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

import math
import numpy as np
from scipy.spatial.transform import Rotation


def normalize_angle(angle):
  """Normalize angle
  """
  a = (angle + math.pi) % (2.0 * math.pi)
  if a < 0.0:
    a += (2.0 * math.pi)
  return a - math.pi


def get_transform_matrix(quat, vector):
  """Get transform matrix from quaternion and vector

  Args:
      quat: [qx, qy, qz, qw]
      vector: [x, y, z]
  """
  rotation_matrix = Rotation.from_quat(quat)
  transform_vector = np.array(vector).reshape(-1, 1)
  rotation = rotation_matrix.as_matrix()
  transform_matrix = np.concatenate([rotation, transform_vector], axis=1)
  bottom = np.array([0, 0, 0, 1]).reshape(1, -1)
  transform_matrix = np.concatenate([transform_matrix, bottom], axis=0)
  return transform_matrix


def get_quat_and_vector(transform_matrix):
  """Get quaternion and vector from transform matrix

  Args:
      transform_matrix: transformation matrix
  """
  rotation_mat = transform_matrix[0:3, 0:3]
  vector = transform_matrix[0:3, 3]
  rotation = Rotation.from_matrix(rotation_mat)
  quat = rotation.as_quat()
  return quat, vector


def euler_to_rotation_matrix(theta1, theta2, theta3, order='xyz'):
  """Euler to rotation matrix
  """
  c1 = np.cos(theta1)
  s1 = np.sin(theta1)
  c2 = np.cos(theta2)
  s2 = np.sin(theta2)
  c3 = np.cos(theta3)
  s3 = np.sin(theta3)

  if order == 'xzx':
    matrix = np.array([[c2, -c3*s2, s2*s3],
                       [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
                       [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]])
  elif order == 'xyx':
    matrix = np.array([[c2, s2*s3, c3*s2],
                       [s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1],
                       [-c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3]])
  elif order == 'yxy':
    matrix = np.array([[c1*c3-c2*s1*s3, s1*s2, c1*s3+c2*c3*s1],
                       [s2*s3, c2, -c3*s2],
                       [-c3*s1-c1*c2*s3, c1*s2, c1*c2*c3-s1*s3]])
  elif order=='yzy':
    matrix = np.array([[c1*c2*c3-s1*s3, -c1*s2, c3*s1+c1*c2*s3],
                      [c3*s2, c2, s2*s3],
                      [-c1*s3-c2*c3*s1, s1*s2, c1*c3-c2*s1*s3]])
  elif order=='zyz':
    matrix = np.array([[c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2],
                      [c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2],
                      [-c3*s2, s2*s3, c2]])
  elif order=='zxz':
    matrix = np.array([[c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1, s1*s2],
                      [c3*s1+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2],
                      [s2*s3, c3*s2, c2]])
  elif order=='xyz':
    matrix = np.array([[c2*c3, -c2*s3, s2],
                      [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                      [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
  elif order=='xzy':
    matrix = np.array([[c2*c3, -s2, c2*s3],
                      [s1*s3+c1*c3*s2, c1*c2, c1*s2*s3-c3*s1],
                      [c3*s1*s2-c1*s3, c2*s1, c1*c3+s1*s2*s3]])
  elif order=='yxz':
    matrix = np.array([[c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1],
                      [c2*s3, c2*c3, -s2],
                      [c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2]])
  elif order=='yzx':
    matrix = np.array([[c1*c2, s1*s3-c1*c3*s2, c3*s1+c1*s2*s3],
                      [s2, c2*c3, -c2*s3],
                      [-c2*s1, c1*s3+c3*s1*s2, c1*c3-s1*s2*s3]])
  elif order=='zyx':
    matrix = np.array([[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
                      [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
                      [-s2, c2*s3, c2*c3]])
  elif order=='zxy':
    matrix = np.array([[c1*c3-s1*s2*s3, -c2*s1, c1*s3+c3*s1*s2],
                      [c3*s1+c1*s2*s3, c1*c2, s1*s3-c1*c3*s2],
                      [-c2*s3, s2, c2*c3]])
  return matrix


def is_rotation_matrix(R):
  """Determine whether R is rotation matrix
  """
  Rt = np.transpose(R)
  should_be_identity = np.dot(Rt, R)
  I = np.identity(3, dtype = R.dtype)
  n = np.linalg.norm(I - should_be_identity)
  return n < 1e-6


def rotation_matrix_to_euler(R):
  """Transform rotation matrix to euler
  """
  assert(is_rotation_matrix(R))
  sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
  singular = sy < 1e-6
  if not singular :
    x = math.atan2(R[2,1] , R[2,2])
    y = math.atan2(-R[2,0], sy)
    z = math.atan2(R[1,0], R[0,0])
  else :
    x = math.atan2(-R[1,2], R[1,1])
    y = math.atan2(-R[2,0], sy)
    z = 0
  return np.array([x, y, z])


class Euler(object):
  def __init__(self, roll, pitch, yaw) -> None:
    """Init
    """
    self.roll = roll
    self.pitch = pitch
    self.yaw = yaw

  def to_quaternion(self):
    """Transform euler to quaternion
    """
    cr = math.cos(self.roll * 0.5)
    sr = math.sin(self.roll * 0.5)
    cp = math.cos(self.pitch * 0.5)
    sp = math.sin(self.pitch * 0.5)
    cy = math.cos(self.yaw * 0.5)
    sy = math.sin(self.yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return Quaternion(qw, qx, qy, qz)


class Quaternion(object):
  def __init__(self, w, x, y, z) -> None:
    """Init
    """
    self.w = w
    self.x = x
    self.y = y
    self.z = z

  def to_euler(self):
    """Quaternion to euler
    """
    t0 = 2 * (self.w * self.x + self.y * self.z)
    t1 = 1 - 2 * (self.x * self.x + self.y * self.y)
    roll = math.atan2(t0, t1)

    t2 = 2 * (self.w * self.y - self.z * self.x)
    pitch = math.asin(t2)

    t3 = 2 * (self.w * self.z + self.x * self.y)
    t4 = 1 - 2 * (self.y * self.y + self.z * self.z)
    yaw = math.atan2(t3, t4)
    return Euler(roll, pitch, yaw)

  def __mul__(self, other):
    """Quaternion multiplication
    """
    w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
    x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
    y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
    z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
    return Quaternion(w, x, y, z)

  def __str__(self) -> str:
    """Transform quaternion to string
    """
    return "w: {}, x: {}, y: {}, z: {}".format(self.w, self.x, self.y, self.z)
