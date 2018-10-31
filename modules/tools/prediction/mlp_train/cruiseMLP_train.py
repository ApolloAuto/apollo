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
from torch.utils.data import Dataset, DataLoader

from sklearn.model_selection import train_test_split

from common.configure import parameters

# Constants
dim_input = parameters['cruise_mlp']['dim_input']
dim_hidden_1 = parameters['cruise_mlp']['dim_hidden_1']
dim_hidden_2 = parameters['cruise_mlp']['dim_hidden_2']
dim_output = parameters['cruise_mlp']['dim_output']

# CUDA setup
cuda_is_available = torch.cuda.is_available()

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



class FCNN_CNN1D(torch.nn.Module):
    def __init__(self):
        super(FCNN_CNN1D, self).__init__()
        self.lane_feature_conv = torch.nn.Sequential(\
                            nn.Conv1d(6, 10, 3, stride=1),\
                            nn.ReLU(),\
                            nn.Conv1d(10, 16, 3, stride=2),\
                            nn.ReLU(),\
                            nn.Conv1d(16, 25, 3, stride=2),\
                            )
        self.lane_feature_maxpool = nn.MaxPool1d(3)
        self.lane_feature_avgpool = nn.AvgPool1d(3)
        self.lane_feature_dropout = nn.Dropout(0.0)

        self.obs_feature_fc = torch.nn.Sequential(\
                            nn.Linear(23, 17),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.0),\
                            nn.Linear(17, 12),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.0),\
                            )

        self.classify = torch.nn.Sequential(\
                            nn.Linear(123, 66),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.3),\

                            nn.Linear(66, 48),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.1),\

                            nn.Linear(48, 11),\
                            nn.Sigmoid(),\
                            nn.Dropout(0.1),\

                            nn.Linear(11, 1),\
                            nn.Sigmoid()
                                            )
        self.regress = torch.nn.Sequential(\
                            nn.Linear(124, 77),\
                            nn.ReLU(),\
                            nn.Dropout(0.2),\

                            nn.Linear(77, 46),\
                            nn.ReLU(),\
                            nn.Dropout(0.2),\

                            nn.Linear(46, 12),\
                            nn.ReLU(),\
                            nn.Dropout(0.1),\

                            nn.Linear(12, 1),\
                            nn.ReLU()
                                            )
    def forward(self, x):
        lane_fea = x[:,23:]
        lane_fea = lane_fea.view(lane_fea.size(0), 6, 30)
        obs_fea = x[:,:23]

        lane_fea = self.lane_feature_conv(lane_fea)
        lane_fea_max = self.lane_feature_maxpool(lane_fea)
        lane_fea_avg = self.lane_feature_avgpool(lane_fea)

        lane_fea = torch.cat([lane_fea_max.view(lane_fea_max.size(0),-1), \
                              lane_fea_avg.view(lane_fea_avg.size(0),-1)], 1)
        lane_fea = self.lane_feature_dropout(lane_fea)

        #obs_fea = self.obs_feature_fc(obs_fea)
        #print (lane_fea.shape)
        tot_fea = torch.cat([lane_fea, obs_fea], 1)
        out_c = self.classify(tot_fea)
        out_r = self.regress(torch.cat([tot_fea, out_c], 1))

        return out_c, out_r

'''
Custom defined loss function that lumps the loss of classification and
of regression together.
'''
def loss_fn(c_pred, r_pred, target):
    loss_C = nn.BCELoss()
    loss_R = nn.MSELoss()
    loss = loss_C(c_pred, target[:,0].view(target.shape[0],1))
    #loss = 4 * loss_C(c_pred, target[:,0].view(target.shape[0],1)) + \
          #loss_R((target[:,1] < 10.0).float().view(target.shape[0],1) * r_pred + \
          #        (target[:,1] >= 10.0).float().view(target.shape[0],1) * target[:,1].view(target.shape[0],1), \
          #        target[:,1].view(target.shape[0],1))
          #loss_R((target[:,0] == True).float().view(target.shape[0],1) * r_pred + \
          #        (target[:,0] == False).float().view(target.shape[0],1) * target[:,1].view(target.shape[0],1), \
          #        target[:,1].view(target.shape[0],1))
    return loss

# ========================================================================
# Data Loading and preprocessing

'''
Load the data from h5 file to the numpy format.
(Only works for small datasets that can be entirely loaded into memory)
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
Get the full path of all files under the directory: 'dirName'
'''
def getListOfFiles(dirName):
    listOfFiles = os.listdir(dirName)
    allFiles = list()
    
    for entry in listOfFiles:
        fullPath = os.path.join(dirName, entry)
        if os.path.isdir(fullPath):
            allFiles = allFiles + getListOfFiles(fullPath)
        else:
            allFiles.append(fullPath)
    
    return allFiles

'''
TODO: implement custom collate_fn to incorporate down-sampling function
for certain labels.
'''
def collate_wDownSample(batch):
    return None

'''
If datasets are too large, use Dataloader to load from disk.
'''
class TrainValidDataset(Dataset):
    '''
    Args:
      - root_dir (string): Directory containing all folders with different
        dates, each folder containing .cruise.h5 data files.
    '''
    def __init__(self, root_dir):
        self.root_dir_ = root_dir
        self.list_of_files_ = getListOfFiles(self.root_dir_)
        self.data_size_until_this_file_ = []
        self.dataset_size = 0
        for file in self.list_of_files_:
            with h5py.File(file, 'r') as h5_file:
                data_size = h5_file[list(h5_file.keys())[0]].shape[0]
            self.dataset_size += data_size
            self.data_size_until_this_file_.append(self.dataset_size)
        #print ('Total size of dataset: {}'.format(self.data_size_until_this_file_))

    def __len__(self):
        return self.dataset_size

    def __getitem__(self, index):
        bin_idx = self.FindBin(index, 0, len(self.data_size_until_this_file_)-1)
        with h5py.File(self.list_of_files_[bin_idx], 'r') as h5_file:
            idx_offset = self.data_size_until_this_file_[bin_idx] - \
                         h5_file[list(h5_file.keys())[0]].shape[0]
            data = h5_file[list(h5_file.keys())[0]][index-idx_offset]
        label = data[-dim_output:]
        label[0] = (label[0] > 0.0).astype(float)
        return data[:-dim_output], label

    # Binary search to expedite the data-loading process.
    def FindBin(self, index, start, end):
        if (start==end):
            return start

        mid = int((start+end)/2.0)
        if (self.data_size_until_this_file_[mid] <= index):
            return self.FindBin(index, mid+1, end)
        else:
            return self.FindBin(index, start, mid)
# ========================================================================


# ========================================================================
# Data training and validation

'''
Train the data. (vanilla version without dataloader)
'''
def train_vanilla(train_X, train_y, model, optimizer, epoch, batch_size=2048):
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

        if i % 500 == 0:
            logging.info('Step: {}, train_loss: {}'.format(i, np.mean(loss_history[-100:])))
            print ("Step: {}, training loss: {}".format(i, np.mean(loss_history[-100:])))

    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))
    print ('Epoch: {}. Training Loss: {}'.format(epoch, train_loss))

'''
Train the data. (using dataloader)
'''
def train_dataloader(train_loader, model, optimizer, epoch):
    model.train()

    loss_history = []
    logging.info('Epoch: {}'.format(epoch))
    for i, (inputs, targets) in enumerate(train_loader):
        optimizer.zero_grad()
        if cuda_is_available:
            X = (inputs).float().cuda()
            y = (targets).float().cuda()
        c_pred, r_pred = model(X)
        loss = loss_fn(c_pred, r_pred, y)
        #loss.data[0].cpu().numpy()
        loss_history.append(loss.data[0])
        loss.backward()
        optimizer.step()

        #if i > 100:
        #    break
        if i % 100 == 0:
            logging.info('Step: {}, train_loss: {}'.format(i, np.mean(loss_history[-100:])))
            print ("Step: {}, training loss: {}".format(i, np.mean(loss_history[-100:])))

    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))
    print ('Epoch: {}. Training Loss: {}'.format(epoch, train_loss))


'''
Validation (vanilla version without dataloader)
'''
def validate_vanilla(valid_X, valid_y, model, batch_size=1024):
    model.eval()

    loss_history = []
    valid_correct_class = 0.0
    num_of_data = valid_X.shape[0]
    num_of_batch = int(num_of_data / batch_size) + 1
    for i in range(num_of_batch):
        X = valid_X[i*batch_size: min(num_of_data, (i+1)*batch_size),]
        y = valid_y[i*batch_size: min(num_of_data, (i+1)*batch_size),]
        c_pred, r_pred = model(X)
        valid_loss = loss_fn(c_pred, r_pred, y)
        loss_history.append(valid_loss.data[0])
        valid_correct_class += \
            np.sum((c_pred.data.cpu().numpy() > 0.5).astype(float) == y[:,0].data.cpu().numpy().reshape(c_pred.data.cpu().numpy().shape[0],1))

    valid_classification_accuracy = valid_correct_class / valid_y.data.cpu().numpy().shape[0]
    logging.info('Validation loss: {}. Validation classification accuracy: {}'\
        .format(np.mean(loss_history), valid_classification_accuracy))
    print ('Validation loss: {}. Classification accuracy: {}.'\
        .format(np.mean(loss_history), valid_classification_accuracy))

    return valid_loss

'''
Validation (using dataloader)
'''
def validate_dataloader(valid_loader, model):
    model.eval()

    loss_history = []
    valid_correct_class = 0.0
    total_size = 0

    for i, (X, y) in enumerate(valid_loader):
        total_size += y.shape[0]
        if cuda_is_available:
            X = X.float().cuda()
            y = y.float().cuda()
        c_pred, r_pred = model(X)
        valid_loss = loss_fn(c_pred, r_pred, y)
        loss_history.append(valid_loss.data[0])
        valid_correct_class += \
            np.sum((c_pred.data.cpu().numpy() > 0.5).astype(float) == y[:,0].data.cpu().numpy().reshape(c_pred.data.cpu().numpy().shape[0],1))


    valid_classification_accuracy = valid_correct_class / total_size
    logging.info('Validation loss: {}. Validation classification accuracy: {}'\
        .format(np.mean(loss_history), valid_classification_accuracy))
    print ('Validation loss: {}. Classification accuracy: {}.'\
        .format(np.mean(loss_history), valid_classification_accuracy))

    return valid_loss
# ========================================================================

# ========================================================================
# Main function:

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description=\
        'train neural network based on feature files and save parameters')
    parser.add_argument('train_file', type=str, help='training data (h5)')
    parser.add_argument('valid_file', type=str, help='validation data (h5)')

    args = parser.parse_args()

    '''
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
    model = FCNN_CNN1D()
    print ("The model used is: ")
    print (model)
    learning_rate = 5e-4
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau\
        (optimizer, factor=0.5, patience=3, min_lr=1e-8, verbose=1, mode='min')

    # CUDA set-up:
    cuda_is_available = torch.cuda.is_available()
    if (cuda_is_available):
        print ("Using CUDA to speed up training.")
        model.cuda()
        #X_train = Variable(torch.FloatTensor(X_train).cuda())
        #X_valid = Variable(torch.FloatTensor(X_valid).cuda())
        #y_train = Variable(torch.FloatTensor(y_train).cuda())
        #y_valid = Variable(torch.FloatTensor(y_valid).cuda())

    # Model training:
    for epoch in range(100):
        train(X_train, y_train, model, optimizer, epoch)
        valid_loss = validate(X_valid, y_valid, model)
        scheduler.step(valid_loss)
        torch.save(model.state_dict(), './cruiseMLP_saved_model.pt')
    '''

    train_dir = args.train_file
    valid_dir = args.valid_file

    model = FCNN_CNN1D()
    learning_rate = 1e-3
    optimizer = optim.Adam(model.parameters(), lr=learning_rate)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau\
        (optimizer, factor=0.3, patience=2, min_lr=1e-8, verbose=1, mode='min')
    if (cuda_is_available):
        print ('Using CUDA to speed up training.')
        model.cuda()

    train_dataset = TrainValidDataset(train_dir)
    valid_dataset = TrainValidDataset(valid_dir)

    train_loader  = DataLoader(train_dataset, batch_size=2048, num_workers=8, shuffle=True, pin_memory=True)
    valid_loader = DataLoader(valid_dataset, batch_size=2048, num_workers=8, pin_memory=True)

    for epoch in range(100):
        train_dataloader(train_loader, model, optimizer, epoch)
        valid_loss = validate_dataloader(valid_loader, model)
        scheduler.step(valid_loss)

# ========================================================================
