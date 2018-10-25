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
                            nn.Linear(dim_input, dim_hidden_1),\
                            nn.ReLU(),\
                            nn.Dropout(0.3),\

                            nn.Linear(dim_hidden_1, dim_hidden_2),\
                            nn.ReLU(),\
                            nn.Dropout(0.3),\

                            nn.Linear(dim_hidden_2, 1),\
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

def loss_fn(c_pred, r_pred, y_train):
    loss_C = nn.BCELoss()
    loss_R = nn.MSELoss()
    loss = loss_C(c_pred, y_train[:,0].view(y_train.shape[0],1)) + \
           loss_R((y_train[:,0] == True).float().view(y_train.shape[0],1) * r_pred + \
                  (y_train[:,0] == False).float().view(y_train.shape[0],1) * y_train[:,1].view(y_train.shape[0],1), \
                  y_train[:,1].view(y_train.shape[0],1))
    return loss

def train(train_X, train_y, model, optimizer, epoch, batch_size=1024):
    model.train()

    loss_history = []
    logging.info('Epoch: {}'.format(epoch))
    num_of_data = train_X.shape[0]
    num_of_batch = (num_of_data / batch_size) + 1
    for i in range(num_of_batch):
        optimizer.zero_grad()
        X = train_X[i*batch_size: min(num_of_data, (i+1)*batch_size),]
        y = train_y[i*batch_size: min(num_of_data, (i+1)*batch_size),]
        c_pred, r_pred = model(X)
        loss = loss_fn(c_pred, r_pred, train_y)
        #loss.data[0].cpu().numpy()
        loss_history.append(loss.data[0])
        loss.backward()
        optimizer.step()

        if i % 100 == 0:
            logging.info('Step: {}, train_loss: {}'.format(i, np.mean(loss_history[-100:])))

    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))




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

    # CUDA set-up:
    cuda_is_available = torch.cuda.is_available()
    if (cuda_is_available):
        X_train = Variable(torch.FloatTensor(X_train).cuda())
        X_valid = Variable(torch.FloatTensor(X_valid).cuda())
        y_train = Variable(torch.FloatTensor(y_train).cuda())
        y_valid = Variable(torch.FloatTensor(y_valid).cuda())
        model.cuda()

    # Model training:






'''
    model = setup_model()

    model.fit(X_train, Y_trainc, shuffle=True, nb_epoch=20, batch_size=32)
    print ("Model trained success.")

    X_test = (X_test - param_norm[0]) / param_norm[1]

    score = model.evaluate(X_test, Y_testc)
    print ("\nThe accuracy on testing dat is", score[1])

    logging.info("Test data loss: {}, accuracy: {} ".format(
        score[0], score[1]))
    Y_train_hat = model.predict_classes(X_train, batch_size=32)
    Y_test_hat = model.predict_proba(X_test, batch_size=32)
    logging.info("## Training Data:")
    evaluate_model(Y_train, Y_train_hat)
    for thres in [x / 100.0 for x in range(20, 80, 5)]:
        logging.info("##threshond = {} Testing Data:".format(thres))
        performance = evaluate_model(Y_test, Y_test_hat > thres)
    performance['accuracy'] = [score[1]]

    print ("\nFor more detailed evaluation results, please refer to", \
          evaluation_log_path + ".log")

    model_path = os.path.join(os.getcwd(), "mlp_model.bin")
    save_model(model, param_norm, model_path)
    print ("Model has been saved to", model_path)
'''