#!/usr/bin/env python

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
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
@requirement:
    pytorch 0.4.1
"""

import os
import h5py
import numpy as np
import logging
import argparse

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from torch.autograd import Variable

from sklearn.model_selection import train_test_split

from common.configure import parameters

# Constants
dim_input = parameters['cruise_mlp']['dim_input']
dim_hidden_1 = parameters['cruise_mlp']['dim_hidden_1']
dim_hidden_2 = parameters['cruise_mlp']['dim_hidden_2']
dim_output = parameters['cruise_mlp']['dim_output']

#evaluation_log_path = os.path.join(os.getcwd(), "evaluation_report")
#common.log.init_log(evaluation_log_path, level=logging.DEBUG)

'''
Model definition:
    - Fully-connected layers for classification and regression, respectively.
    - It will compute a classification score indicating the probability
      of the obstacle choosing the given lane.
    - It will also compute a time indicating how soon the obstacle will reach
      the center of the given lane.
'''
class FullyConn_NN(torch.nn.Module):
    def __init__(self):
        super(FullyConn_NN, self).__init__()
        self.classify = torch.nn.Sequential(\
                            nn.Linear(83, 55),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.3),\

                            nn.Linear(55, 23),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.2),\

                            nn.Linear(23, 11),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.3),\

                            nn.Linear(11, 5),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.0),\

                            nn.Linear(5, 1),\
                            nn.Sigmoid()
                                            )
        self.regress = torch.nn.Sequential(\
                            nn.Linear(dim_input, dim_hidden_1),\
                            nn.ReLU(),\
                            nn.Dropout(0.1),\
                              
                            nn.Linear(dim_hidden_1, dim_hidden_2),\
                            nn.ReLU(),\
                            nn.Dropout(0.1),\
                               
                            nn.Linear(dim_hidden_2, 1),\
                            nn.ReLU()
                                            )
    def forward(self, x):
        out_c = self.classify(x)
        out_r = self.regress(x)
        return out_c, out_r


class CNN1D_Lane(torch.nn.Module):
    def __init__(self):
        super(FullyConn_NN, self).__init__()
        self.classify = torch.nn.Sequential(\
                            nn.Linear(83, 55),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.3),\

                            nn.Linear(55, 23),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.2),\

                            nn.Linear(23, 11),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.3),\

                            nn.Linear(11, 5),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.0),\

                            nn.Linear(5, 1),\
                            nn.Sigmoid()
                                            )
        self.regress = torch.nn.Sequential(\
                            nn.Linear(dim_input, dim_hidden_1),\
                            nn.ReLU(),\
                            nn.Dropout(0.1),\
                              
                            nn.Linear(dim_hidden_1, dim_hidden_2),\
                            nn.ReLU(),\
                            nn.Dropout(0.1),\
                               
                            nn.Linear(dim_hidden_2, 1),\
                            nn.ReLU()
                                            )
    def forward(self, x):
        out_c = self.classify(x)
        out_r = self.regress(x)
        return out_c, out_r



'''
Load the data from h5 file to the numpy format
'''
def load_data(filename):
    
    if not (os.path.exists(filename)):
        logging.error("file: {}, does not exist".format(filename))
        os._exit(1)
    if os.path.splitext(filename)[1] != '.h5':
        logging.error("file: {} is not an hdf5 file".format(filename))
        os._exit(1)

    samples = dict()
    h5_file = h5py.File(filename, 'r')
    for key in h5_file.keys():
        samples[key] = h5_file[key][:]

    print("load file success")
    return samples['data']

'''
Preprocess the data:
    - separate input X and output y
    - process output label from {-1,0,1,2} to {0,1}
    - shuffle data
'''
def data_preprocessing(data):
    X = data[:, :dim_input]
    y = data[:, -dim_output:]
    y[:, 0] = (y[:, 0] > 0).astype(float)

    X_new, X_dummy, y_new, y_dummy = train_test_split(X, y, test_size=0.0, random_state=233)

    return X_new, y_new

'''
Custom defined loss function that lumps the loss of classification and
of regression together.
'''
def loss_fn(c_pred, r_pred, target):
    loss_C = nn.BCELoss()
    loss_R = nn.MSELoss()
    loss = loss_C(c_pred, target[:,0].view(target.shape[0],1))
    #loss = 4 * loss_C(c_pred, target[:,0].view(target.shape[0],1)) + \
    #      loss_R((target[:,0] == True).float().view(target.shape[0],1) * r_pred + \
    #              (target[:,0] == False).float().view(target.shape[0],1) * target[:,1].view(target.shape[0],1), \
    #              target[:,1].view(target.shape[0],1))
    return loss

'''
Train the data.
'''
def train(train_X, train_y, model, optimizer, epoch, batch_size=256):
    model.train()

    loss_history = []
    logging.info('Epoch: {}'.format(epoch))
    num_of_data = train_X.shape[0]
    num_of_batch = int(num_of_data / batch_size) + 1
    for i in range(num_of_batch):
        optimizer.zero_grad()
        X = train_X[i*batch_size: min(num_of_data, (i+1)*batch_size),]
        y = train_y[i*batch_size: min(num_of_data, (i+1)*batch_size),]
        c_pred, r_pred = model(X)
        loss = loss_fn(c_pred, r_pred, y)
        #loss.data[0].cpu().numpy()
        loss_history.append(loss.data[0])
        loss.backward()
        optimizer.step()

        if i % 1000 == 0:
            logging.info('Step: {}, train_loss: {}'.format(i, np.mean(loss_history[-100:])))
            print ("Step: {}, training loss: {}".format(i, np.mean(loss_history[-100:])))

    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))
    print ('Epoch: {}. Training Loss: {}'.format(epoch, train_loss))


'''
Validation
'''
def validate(valid_X, valid_y, model):
    model.eval()

    c_pred, r_pred = model(valid_X)
    valid_loss = loss_fn(c_pred, r_pred, valid_y)
    valid_classification_accuracy = \
        np.sum((c_pred.data.cpu().numpy() > 0.5).astype(float) == valid_y[:,0].data.cpu().numpy().reshape(c_pred.data.cpu().numpy().shape[0],1)) / c_pred.data.cpu().numpy().shape[0]

    logging.info('Validation loss: {}. Validation classification accuracy: {}'\
        .format(valid_loss, valid_classification_accuracy))
    print ('Validation loss: {}. Classification accuracy: {}.'\
        .format(valid_loss, valid_classification_accuracy))

    return valid_loss


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description=\
        'train neural network based on feature files and save parameters')
    parser.add_argument('train_file', type=str, help='training data (h5)')
    parser.add_argument('valid_file', type=str, help='validation data (h5)')

    args = parser.parse_args()
    train_file = args.train_file
    valid_file = args.valid_file

    train_data = load_data(train_file)
    valid_data = load_data(valid_file)

    print ("Data load success.")
    print ("Training data size = ", train_data.shape)
    print ("Validation data size = ", valid_data.shape)

    # Data preprocessing
    X_train, y_train = data_preprocessing(train_data)
    X_valid, y_valid = data_preprocessing(valid_data)

    # Model declaration
    model = FullyConn_NN()
    print ("The model used is: ")
    print (model)
    learning_rate = 3e-3
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau\
        (optimizer, factor=0.3, patience=3, min_lr=1e-8, verbose=1, mode='min')

    # CUDA set-up:
    cuda_is_available = torch.cuda.is_available()
    if (cuda_is_available):
        print ("Using CUDA to speed up training.")
        X_train = Variable(torch.FloatTensor(X_train).cuda())
        X_valid = Variable(torch.FloatTensor(X_valid).cuda())
        y_train = Variable(torch.FloatTensor(y_train).cuda())
        y_valid = Variable(torch.FloatTensor(y_valid).cuda())
        model.cuda()

    # Model training:
    for epoch in range(100):
        train(X_train, y_train, model, optimizer, epoch)
        valid_loss = validate(X_valid, y_valid, model)
        scheduler.step(valid_loss)
