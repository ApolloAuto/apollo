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

import logging
import math
import numpy as np

import sklearn
from sklearn.model_selection import train_test_split
from sklearn.utils import class_weight

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader, sampler


def cuda(x):
    #return x
    if isinstance(x, (list, tuple)):
        return [cuda(y) for y in x]
    return x.cuda() if torch.cuda.is_available() else x

def train_vanilla(train_X, train_y, model, loss, optimizer, epoch,
                  batch_preprocess, batch_size=1024, print_period=100):
    model.train()

    loss_history = []
    logging.info('Epoch: {}:'.format(epoch))
    print('Epoch: {}:'.format(epoch))
    num_of_data = train_X.shape[0]
    num_of_batch = math.ceil(num_of_data / batch_size)
    pred_y = []
    for i in range(num_of_batch):
        optimizer.zero_grad()
        X = train_X[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        y = train_y[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        X, y = batch_preprocess(X, y)
        pred = model(X)
        train_loss = loss.loss_fn(pred, y)
        loss_history.append(train_loss.item())
        train_loss.backward()
        optimizer.step()

        pred = pred.detach().cpu().numpy()
        pred_y.append(pred)

        if (i > 0) and (i % print_period == 0):
            logging.info('   Step: {}, training loss: {}'.format(
                i, np.mean(loss_history[-print_period:])))
            print ('   Step: {}, training loss: {}'.format(
                i, np.mean(loss_history[-print_period:])))

    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))
    print('Training Loss: {}'.format(train_loss))
    loss.loss_info(pred_y, train_y)

def valid_vanilla(valid_X, valid_y, model, loss, batch_preprocess,
                  batch_size=1024):
    model.eval()

    loss_history = []
    num_of_data = valid_X.shape[0]
    num_of_batch = math.ceil(num_of_data / batch_size)
    pred_y = []
    for i in range(num_of_batch):
        X = valid_X[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        y = valid_y[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        X, y = batch_preprocess(X, y)
        pred = model(X)
        valid_loss = loss.loss_fn(pred, y)
        loss_history.append(valid_loss.item())

        pred = pred.detach().cpu().numpy()
        pred_y.append(pred)

    valid_loss = np.mean(loss_history)
    logging.info('Validation loss: {}.'.format(valid_loss))
    print ('Validation loss: {}.'.format(valid_loss))
    loss.loss_info(pred_y, valid_y)

    return valid_loss

def train_valid_vanilla(train_X, train_y, valid_X, valid_y, model, loss,
                        optimizer, scheduler, epochs, save_name, batch_preprocess,
                        train_batch=1024, print_period=100, valid_batch=1024):
    best_valid_loss = float('+inf')
    for epoch in range(1, epochs+1):
        train_vanilla(train_X, train_y, model, loss, optimizer, epoch,
                      batch_preprocess, train_batch, print_period)
        valid_loss = valid_vanilla(valid_X, valid_y, model, loss,
                                   batch_preprocess, valid_batch)

        scheduler.step(valid_loss)

        # TODO(jiacheng): add early stopping mechanism
        if valid_loss < best_valid_loss:
            best_valid_loss = valid_loss
            # save_checkpoint()

    return model

def train_dataloader(train_loader, model, loss, optimizer, epoch, print_period=100):
    with torch.autograd.set_detect_anomaly(True):
        model.train()

        loss_history = []
        logging.info('Epoch: {}:'.format(epoch))
        print('Epoch: {}:'.format(epoch))
        for i, (X, y) in enumerate(train_loader):
            optimizer.zero_grad()
            X, y = cuda(X), cuda(y)
            pred = model(X)
            train_loss = loss.loss_fn(pred, y)
            loss_history.append(train_loss.item())
            train_loss.backward()
            optimizer.step()

            if (i > 0) and (i % print_period == 0):
                logging.info('   Step: {}, training loss: {}'.format(
                    i, np.mean(loss_history[-print_period:])))
                print ('   Step: {}, training loss: {}'.format(
                    i, np.mean(loss_history[-print_period:])))

        train_loss = np.mean(loss_history)
        logging.info('Training loss: {}'.format(train_loss))
        print('Training Loss: {}'.format(train_loss))

def valid_dataloader(valid_loader, model, loss):
    model.eval()

    loss_history = []
    for i, (X, y) in enumerate(valid_loader):
        X, y = cuda(X), cuda(y)
        pred = model(X)
        valid_loss = loss.loss_fn(pred, y)
        loss_history.append(valid_loss.item())
        valid_loss_info = loss.loss_info(pred, y).item()
        print (valid_loss_info)
        logging.info('Validation avg displacement = {}'.format(valid_loss_info))

    valid_loss = np.mean(loss_history)
    logging.info('Validation loss: {}.'.format(valid_loss))
    print ('Validation loss: {}.'.format(valid_loss))

    return valid_loss

def train_valid_dataloader(train_loader, valid_loader, model, loss, optimizer,
                           scheduler, epochs, save_name, print_period=100):
    best_valid_loss = float('+inf')
    for epoch in range(1, epochs+1):
        train_dataloader(train_loader, model, loss, optimizer, epoch, print_period)
        valid_loss = valid_dataloader(valid_loader, model, loss)

        scheduler.step(valid_loss)

        # TODO(jiacheng): add early stopping mechanism
        if valid_loss < best_valid_loss:
            best_valid_loss = valid_loss
            # save_checkpoint()

    return model

# TODO(jiacheng): implement this
def save_checkpoint():
    return
