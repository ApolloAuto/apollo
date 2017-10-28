#!/usr/bin/env python

###############################################################################
# Copyright 2017 The Apollo Authors. All Rights Reserved.
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
This module trains end to end model and predict .
"""
import os
import argparse
import glob
import numpy as np
import h5py
import tensorflow as tf
import cv2
import math
import random
from keras.optimizers import Adam
import steering_model
import acc_model


def filter(c, v):
    """
    :param c: curvature
    :param v: speed
    :return: Ture or False
    """
    return v > 1 and c < 0.5


def run_train_steering(image_paths, attr_paths, nb_epoch, batch_size, log_type):
    """
    :param image_paths: input image path
    :param attr_paths: input attr path
    :param nb_epoch: epochs size
    :param batch_size: batch size
    :param log_type: log type
    """

    def generate_data_steering():
        """
        :return: (img,curv)
        """
        buffer_i = []
        buffer_c = []
        for _ in range(nb_epoch):
            for path0, path1 in zip(image_paths, attr_paths):
                with h5py.File(path1, 'r') as f:
                    attrs = f['attrs'][:]
                with h5py.File(path0, 'r') as f:
                    for i, t in enumerate(f):
                        img = cv2.imdecode(f[t][:], 1) / 127.5 - 1.0
                        curv = attrs[i][4]
                        speed = math.sqrt(math.pow(attrs[i][1], 2) + math.pow(attrs[i][2], 2))
                        if filter(curv, speed) and (curv > 0.0001 or random.randint(1, 10) == 1):
                            buffer_i.append(img)
                            buffer_c.append(curv)
                            if len(buffer_i) >= batch_size:
                                yield np.asarray(buffer_i), np.asarray(buffer_c)
                                buffer_i = []
                                buffer_c = []

        if len(buffer_i) > 0:
            yield np.asarray(buffer_i), np.asarray(buffer_c)
        raise StopIteration

    model = steering_model.get_model((320, 320, 3))
    model.compile(loss='mse', optimizer=Adam(lr=0.00001))

    train_log = open('logs/{}_steering.log'.format(log_type), 'w')
    step = 0
    for i, c in generate_data_steering():
        step += 1
        history = model.fit(i, c, batch_size=batch_size, epochs=1)
        train_log.write('steering Iteration {}, loss = {} \n' \
                        .format(step, history.history['loss'][0]))
        if step % 5000 == 0:
            model.save_weights('models/steering_model.{}'.format(step))
    # save last step
    if step % 5000 != 0:
        model.save_weights('models/steering_model.{}'.format(step))

    train_log.write('steering train end')
    train_log.close()


def run_train_acc(image_paths, attr_paths, nb_epoch, batch_size, log_type):
    """
    :param image_paths: input image path
    :param attr_paths: input attr path
    :param nb_epoch: epochs size
    :param batch_size: batch size
    :param log_type: log type
    """

    def generate_data_acc():
        """
        :return: (img,acc)
        """
        time_step = 5
        t_sep = 0.25

        batch_img = np.zeros((batch_size, time_step, 320, 320, 3), dtype='uint8')
        batch_acc = np.zeros((batch_size, 1), dtype='float32')
        step_img = np.zeros((time_step, 320, 320, 3), dtype='uint8')
        step_v  = [0] * time_step
        step_t = [0] * time_step
        for _ in range(nb_epoch):
            for path0, path1 in zip(image_paths, attr_paths):
                batch_cnt = 0
                step_cnt = 0
                with h5py.File(path1, 'r') as f:
                    attrs = f['attrs'][:]
                with h5py.File(path0, 'r') as f:
                    for i, t in enumerate(f):
                        img = cv2.imdecode(f[t][:], 1) / 127.5 - 1.0
                        curv = attrs[i][4]
                        v = math.sqrt(attrs[i][1] ** 2 + attrs[i][2] ** 2)

                        if filter(curv, v) and (step_cnt != time_step - 1):
                            step_img[step_cnt] = img
                            step_v[step_cnt] = v
                            step_t[step_cnt] = t
                            step_cnt = (step_cnt + 1) % time_step

                        else:
                            if i != len(attrs) - 1:
                                next_curv = attrs[i + 1][4]
                                next_speed = math.sqrt(attrs[i + 1][1] ** 2 + attrs[i + 1][2] ** 2)

                                if not filter(next_curv, next_speed):
                                    step_cnt = 0
                                    continue
                                else:
                                    acc = (next_speed - step_v[3]) / t_sep
                                    step_img[step_cnt] = img
                                    batch_img[batch_cnt] = step_img
                                    batch_acc[batch_cnt] = acc
                                    step_cnt = (step_cnt + 1) % time_step
                                    batch_cnt = (batch_cnt + 1) % batch_size
                                    if batch_cnt == 0:
                                        yield batch_img, batch_acc
                                        batch_cnt = 0
                                        step_cnt = 0
                            else:
                                break

        raise StopIteration

    model = acc_model.get_model((5, 320, 320, 3))
    train_log = open('logs/{}_acc.log'.format(log_type), 'w')

    step = 0
    for i, a in generate_data_acc():
        step += 1
        history = model.fit(i, a, batch_size=batch_size, epochs=1)
        train_log.write('acc Iteration {}, loss = {} \n' \
                        .format(step, history.history['loss'][0]))
        if step % 5000 == 0:
            model.save_weights('models/acc_model.{}'.format(step))

    # save last step
    if step % 5000 != 0:
        model.save_weights('models/acc_model.{}'.format(step))

    train_log.write('acc train end')
    train_log.close()


def run_predict(steering_model_path, acc_model_path, image_paths, output_path, log_type):

    """
    :param steering_model_path: steering model
    :param acc_model_path: acc model
    :param image_paths: test image path
    :param output_path: predict output path
    :param log_type: log type
    """
    pred_log = open('logs/{}.log'.format(log_type), 'w')
    model = steering_model.get_model((320, 320, 3))
    model.load_weights(steering_model_path)

    time_list = []
    pred_c_list = []
    for path in image_paths:
        pred_log.write('run steering predict: {}...\n'.format(path))
        with h5py.File(path, 'r') as f:
            for t in f:
                img = cv2.imdecode(f[t][:], 1) / 127.5 - 1.0
                img = img.reshape((1,) + img.shape)
                pred_c = model.predict(img)
                time_list.append(float(t))
                pred_c_list.append(pred_c[0][0])

    #predict acc
    steps_size = 5
    model = acc_model.get_model((steps_size, 320, 320, 3))
    model.load_weights(acc_model_path)

    pred_a_list = []
    buffer_i = []
    for path in image_paths:
        pred_log.write('run acc predict: {}...\n'.format(path))
        with h5py.File(path, 'r') as f:
            for t in f:
                img = cv2.imdecode(f[t][:], 1) / 127.5 - 1.0
                buffer_i.append(img)
                if len(buffer_i) < steps_size: pred_a_list.append(0)
                if len(buffer_i) >= steps_size:
                    pred_a = model.predict(
                             np.asarray(buffer_i).reshape((1,)+np.asarray(buffer_i).shape)
                             )
                    pred_a_list.append(pred_a[0][0])
                    buffer_i.pop(0)

    pred_log.write('write predictions...\n')
    with h5py.File(output_path, 'w') as f:
        data = f.create_dataset('attrs', shape=(len(time_list), 3), dtype=np.float64)
        data[:, 0] = np.asarray(time_list)
        data[:, 1] = np.asarray(pred_c_list)
        data[:, 2] = np.asarray(pred_a_list)
    pred_log.close()


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='run_model')
    parser.add_argument('--type', choices=['train', 'predict'], required=True)
    parser.add_argument('--logtype', required=True)
    parser.add_argument('--input_dir', required=True)
    parser.add_argument('--nb_epoch', type=int, default=1)
    parser.add_argument('--batch_size', type=int, default=100)
    parser.add_argument('--steering_model_path')
    parser.add_argument('--acc_model_path')
    parser.add_argument('--output_path')
    args = parser.parse_args()

    config = tf.ConfigProto()
    config.gpu_options.allow_growth=True

    if args.type == 'train':
        image_paths = sorted(glob.glob(os.path.join(args.input_dir, 'image', '*.h5')))
        attr_paths = sorted(glob.glob(os.path.join(args.input_dir, 'attr', '*.h5')))
        with tf.Session(config=config):
            run_train_steering(image_paths, attr_paths,
                              args.nb_epoch, args.batch_size, args.logtype)
        with tf.Session(config=config):
            run_train_acc(image_paths, attr_paths, args.nb_epoch, args.batch_size, args.logtype)
    else:
        image_paths = sorted(glob.glob(os.path.join(args.input_dir, 'image', '*.h5')))
        with tf.Session(config=config):
            run_predict(args.steering_model_path, args.acc_model_path,
                        image_paths, args.output_path, args.logtype)
