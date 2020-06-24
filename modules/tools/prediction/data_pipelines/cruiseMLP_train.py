#!/usr/bin/env python3

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

import argparse
import logging
import os

from sklearn.model_selection import train_test_split
from sklearn.utils import class_weight
from torch.autograd import Variable
from torch.utils.data import Dataset, DataLoader, sampler
import h5py
import numpy as np
import sklearn
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

from modules.tools.prediction.data_pipelines.common.configure import parameters
from modules.tools.prediction.data_pipelines.cruise_models import FullyConn_NN, FCNN_CNN1D
from modules.tools.prediction.data_pipelines.proto.cruise_model_pb2 import TensorParameter, InputParameter,\
    Conv1dParameter, DenseParameter, ActivationParameter, MaxPool1dParameter,\
    AvgPool1dParameter, LaneFeatureConvParameter, ObsFeatureFCParameter,\
    ClassifyParameter, RegressParameter, CruiseModelParameter


# TODO(panjiacheng): the data-loader part needs to be modified.

# Constants
dim_input = parameters['cruise_mlp']['dim_input']
dim_hidden_1 = parameters['cruise_mlp']['dim_hidden_1']
dim_hidden_2 = parameters['cruise_mlp']['dim_hidden_2']
dim_output = parameters['cruise_mlp']['dim_output']

# Setup
cuda_is_available = torch.cuda.is_available()
logging.basicConfig(filename='training.log', level=logging.INFO)


def load_Conv1dParameter(model, key, stride=1):

    model_pb = Conv1dParameter()

    model_pb.shape.extend(list(model.state_dict()[key+'.weight'].shape))

    model_pb.use_bias = True

    kernel_param = TensorParameter()
    kernel_param.shape.extend(list(model.state_dict()[key+'.weight'].shape))
    kernel_param.data.extend(
        list(model.state_dict()[key+'.weight'].numpy().reshape(-1)))
    model_pb.kernel.CopyFrom(kernel_param)

    bias_param = TensorParameter()
    bias_param.shape.extend(list(model.state_dict()[key+'.bias'].shape))
    bias_param.data.extend(
        list(model.state_dict()[key+'.bias'].numpy().reshape(-1)))
    model_pb.bias.CopyFrom(bias_param)

    model_pb.stride = stride

    return model_pb


def load_DenseParameter(model, key):

    model_pb = DenseParameter()

    model_pb.use_bias = True

    weights_param = TensorParameter()
    weights_param.shape.extend(
        list(model.state_dict()[key+'.weight'].numpy().T.shape))
    weights_param.data.extend(
        list(model.state_dict()[key+'.weight'].numpy().T.reshape(-1)))
    model_pb.weights.CopyFrom(weights_param)

    bias_param = TensorParameter()
    bias_param.shape.extend(
        list(model.state_dict()[key+'.bias'].numpy().shape))
    bias_param.data.extend(list(model.state_dict()[key+'.bias'].numpy()))
    model_pb.bias.CopyFrom(bias_param)

    model_pb.units = model_pb.bias.shape[0]

    return model_pb


def save_FCNN_CNN1D(model, filename):

    model_pb = CruiseModelParameter()

    lane_feature_conv = LaneFeatureConvParameter()
    lane_feature_conv.conv1d_0.CopyFrom(
        load_Conv1dParameter(model, 'lane_feature_conv.0', stride=1))
    lane_feature_conv.activation_1.activation = 'relu'
    lane_feature_conv.conv1d_2.CopyFrom(
        load_Conv1dParameter(model, 'lane_feature_conv.2', stride=2))
    lane_feature_conv.activation_3.activation = 'relu'
    lane_feature_conv.conv1d_4.CopyFrom(
        load_Conv1dParameter(model, 'lane_feature_conv.4', stride=2))

    lane_feature_maxpool = MaxPool1dParameter()
    lane_feature_maxpool.kernel_size = 3
    lane_feature_maxpool.stride = 3

    lane_feature_avgpool = AvgPool1dParameter()
    lane_feature_avgpool.kernel_size = 3
    lane_feature_avgpool.stride = 3

    obs_feature_fc = ObsFeatureFCParameter()
    obs_feature_fc.linear_0.CopyFrom(
        load_DenseParameter(model, 'obs_feature_fc.0'))
    obs_feature_fc.activation_1.activation = 'sigmoid'
    obs_feature_fc.linear_3.CopyFrom(
        load_DenseParameter(model, 'obs_feature_fc.3'))
    obs_feature_fc.activation_4.activation = 'sigmoid'

    classify = ClassifyParameter()
    classify.linear_0.CopyFrom(load_DenseParameter(model, 'classify.0'))
    classify.activation_1.activation = 'sigmoid'
    classify.linear_3.CopyFrom(load_DenseParameter(model, 'classify.3'))
    classify.activation_4.activation = 'sigmoid'
    classify.linear_6.CopyFrom(load_DenseParameter(model, 'classify.6'))
    classify.activation_7.activation = 'sigmoid'
    classify.linear_9.CopyFrom(load_DenseParameter(model, 'classify.9'))
    classify.activation_10.activation = 'sigmoid'

    regress = RegressParameter()
    regress.linear_0.CopyFrom(load_DenseParameter(model, 'regress.0'))
    regress.activation_1.activation = 'relu'
    regress.linear_3.CopyFrom(load_DenseParameter(model, 'regress.3'))
    regress.activation_4.activation = 'relu'
    regress.linear_6.CopyFrom(load_DenseParameter(model, 'regress.6'))
    regress.activation_7.activation = 'relu'
    regress.linear_9.CopyFrom(load_DenseParameter(model, 'regress.9'))
    regress.activation_10.activation = 'relu'

    model_pb.lane_feature_conv.CopyFrom(lane_feature_conv)
    model_pb.lane_feature_maxpool.CopyFrom(lane_feature_maxpool)
    model_pb.lane_feature_avgpool.CopyFrom(lane_feature_avgpool)
    model_pb.obs_feature_fc.CopyFrom(obs_feature_fc)
    model_pb.classify.CopyFrom(classify)
    model_pb.regress.CopyFrom(regress)

    with open(filename, 'wb') as params_file:
        params_file.write(model_pb.SerializeToString())


'''
Custom defined loss function that lumps the loss of classification and
of regression together.
'''


def loss_fn(c_pred, r_pred, target, balance):
    loss_C = nn.BCEWithLogitsLoss(
        pos_weight=torch.FloatTensor([balance]).cuda())  # nn.BCELoss()
    loss_R = nn.MSELoss()

    #loss = loss_C(c_pred, target[:,0].view(target.shape[0],1))
    loss = 4 * loss_C(c_pred, target[:, 0].view(target.shape[0], 1)) + \
        loss_R(((target[:, 2] > 0.0) * (target[:, 2] <= 3.0)).float().view(target.shape[0], 1) * r_pred +
               ((target[:, 2] <= 0.0) + (target[:, 2] > 3.0)).float().view(
                   target.shape[0], 1) * target[:, 2].view(target.shape[0], 1),
               target[:, 2].view(target.shape[0], 1))
    # loss_R((target[:,1] < 10.0).float().view(target.shape[0],1) * r_pred + \
    #        (target[:,1] >= 10.0).float().view(target.shape[0],1) * target[:,1].view(target.shape[0],1), \
    #        target[:,1].view(target.shape[0],1))
    return loss


# ========================================================================
# Helper functions
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
Print the distribution of data labels.
'''


def print_dist(label):
    unique_labels = np.unique(label)
    for l in unique_labels:
        print('Label = {}: {}%'.format(l, np.sum(label == l)/len(label)*100))


# ========================================================================


# ========================================================================
# Data Loading and preprocessing (Non Data-Loader case)


def load_data(filename):
    '''
    Load the data from h5 file to the numpy format.
    (Only for non data-loader case)
    '''
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


def load_npy_data(dir):
    '''
    Load all .npy files under a certain dir;
    merge them together into one;
    return.
    '''


def data_preprocessing(data):
    '''
    Preprocess the data.
    (Only for non data-loader case)
        - separate input X and output y
        - process output label from {-1,0,1,2,3,4} to {0,1}
        - Take out only those meaningful features
        - shuffle data
    '''
    # Various input features separation
    X_obs_old_features = data[:, 0:23]
    X_surround_obs = data[:, -dim_output-8:-dim_output]
    X_obs_now = data[:, 23:32]
    X_obs_hist_5 = data[:, 23:68]
    X_lane = data[:, 68:-dim_output-8]

    # mask out those that don't have any history
    # mask5 = (data[:,53] != 100)

    X = np.concatenate((X_obs_old_features, X_obs_hist_5, X_lane), axis=1)
    # X = X[mask5, :]
    y = data[:, -dim_output:]
    # y = y[mask5, :]

    # Binary classification
    y[:, 0] = (y[:, 0] > 0).astype(float)
    #y[:, 0] = np.logical_and((y[:, 0] > 0), (y[:, 1] < 1.0))

    # Random shuffling
    X_new, X_dummy, y_new, y_dummy = train_test_split(
        X, y, test_size=0.0, random_state=233)

    return X_new, y_new  # , X_dummy, y_dummy

# ========================================================================


# ========================================================================
# Data Loading and preprocessing (Data-Loader case)

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

    def __init__(self, list_of_files):
        self.list_of_files_ = list_of_files
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
        bin_idx = self.FindBin(index, 0, len(
            self.data_size_until_this_file_)-1)
        with h5py.File(self.list_of_files_[bin_idx], 'r') as h5_file:
            idx_offset = self.data_size_until_this_file_[bin_idx] - \
                h5_file[list(h5_file.keys())[0]].shape[0]
            data = h5_file[list(h5_file.keys())[0]][index-idx_offset]
        label = data[-dim_output:]
        label[0] = (label[0] > 0.0).astype(float)
        return data[:-dim_output], label

    # Binary search to expedite the data-loading process.
    def FindBin(self, index, start, end):
        if (start == end):
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


def train_vanilla(train_X, train_y, model, optimizer, epoch, batch_size=2048, balance=1.0):
    model.train()

    loss_history = []
    logging.info('Epoch: {}'.format(epoch+1))
    print('Epoch: {}.'.format(epoch+1))
    num_of_data = train_X.shape[0]
    num_of_batch = int(num_of_data / batch_size) + 1
    pred_y = None
    for i in range(num_of_batch):
        optimizer.zero_grad()
        X = train_X[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        y = train_y[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        c_pred, r_pred = model(X)
        loss = loss_fn(c_pred, r_pred, y, balance)
        loss_history.append(loss.data)
        loss.backward()
        optimizer.step()

        c_pred = c_pred.data.cpu().numpy()
        c_pred = c_pred.reshape(c_pred.shape[0], 1)
        pred_y = np.concatenate((pred_y, c_pred), axis=0) if pred_y is not None \
            else c_pred

        if (i > 0) and (i % 100 == 0):
            logging.info('Step: {}, train_loss: {}'.format(
                i, np.mean(loss_history[-100:])))
            print("Step: {}, training loss: {}".format(
                i, np.mean(loss_history[-100:])))

    pred_y = (pred_y > 0.0)
    train_y = train_y.data.cpu().numpy()
    training_accuracy = sklearn.metrics.accuracy_score(
        train_y[:, 0], pred_y.reshape(-1))
    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))
    logging.info('Training Accuracy: {}.'.format(training_accuracy))

    print('Training Loss: {}. Training Accuracy: {}'
          .format(train_loss, training_accuracy))


'''
Validation (vanilla version without dataloader)
'''


def validate_vanilla(valid_X, valid_y, model, batch_size=2048, balance=1.0, pos_label=1.0):
    model.eval()

    loss_history = []
    num_of_data = valid_X.shape[0]
    num_of_batch = int(num_of_data / batch_size) + 1
    pred_y = None
    for i in range(num_of_batch):
        X = valid_X[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        y = valid_y[i*batch_size: min(num_of_data, (i+1)*batch_size), ]
        c_pred, r_pred = model(X)
        valid_loss = loss_fn(c_pred, r_pred, y, balance)
        loss_history.append(valid_loss.data)

        c_pred = c_pred.data.cpu().numpy()
        c_pred = c_pred.reshape(c_pred.shape[0], 1)

        pred_y = np.concatenate((pred_y, c_pred), axis=0) if pred_y is not None \
            else c_pred

    valid_y = valid_y.data.cpu().numpy()
    valid_auc = sklearn.metrics.roc_auc_score(
        valid_y[:, 0], pred_y.reshape(-1))
    pred_y = (pred_y > 0.0)
    valid_accuracy = sklearn.metrics.accuracy_score(
        valid_y[:, 0], pred_y.reshape(-1))
    valid_precision = sklearn.metrics.precision_score(
        valid_y[:, 0], pred_y.reshape(-1), pos_label=pos_label)
    valid_recall = sklearn.metrics.recall_score(
        valid_y[:, 0], pred_y.reshape(-1), pos_label=pos_label)

    logging.info('Validation loss: {}. Accuracy: {}.\
                  Precision: {}. Recall: {}. AUC: {}.'
                 .format(np.mean(loss_history), valid_accuracy, valid_precision,
                         valid_recall, valid_auc))
    print('Validation loss: {}. Accuracy: {}.\
            Precision: {}. Recall: {}. AUC: {}.'
          .format(np.mean(loss_history), valid_accuracy, valid_precision,
                  valid_recall, valid_auc))

    return np.mean(loss_history)


'''
Train the data. (using dataloader)
'''


def train_dataloader(train_loader, model, optimizer, epoch, balance=1.0):
    model.train()

    loss_history = []
    train_correct_class = 0
    total_size = 0
    logging.info('Epoch: {}'.format(epoch))
    for i, (inputs, targets) in enumerate(train_loader):
        total_size += targets.shape[0]
        optimizer.zero_grad()
        if cuda_is_available:
            X = (inputs).float().cuda()
            y = (targets).float().cuda()
        c_pred, r_pred = model(X)
        loss = loss_fn(c_pred, r_pred, y, balance)
        # loss.data[0].cpu().numpy()
        loss_history.append(loss.data)
        loss.backward()
        optimizer.step()

        train_correct_class += \
            np.sum((c_pred.data.cpu().numpy() > 0.5).astype(float) ==
                   y[:, 0].data.cpu().numpy().reshape(c_pred.data.cpu().numpy().shape[0], 1))

        # if i > 100:
        #    break
        if i % 100 == 0:
            logging.info('Step: {}, train_loss: {}'.format(
                i, np.mean(loss_history[-100:])))
            print("Step: {}, training loss: {}".format(
                i, np.mean(loss_history[-100:])))

    train_loss = np.mean(loss_history)
    logging.info('Training loss: {}'.format(train_loss))
    print('Epoch: {}. Training Loss: {}'.format(epoch, train_loss))


'''
Validation (using dataloader)
'''


def validate_dataloader(valid_loader, model, balance=1.0):
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
        valid_loss = loss_fn(c_pred, r_pred, y, balance)
        loss_history.append(valid_loss.data)
        valid_correct_class += \
            np.sum((c_pred.data.cpu().numpy() > 0.5).astype(float) ==
                   y[:, 0].data.cpu().numpy().reshape(c_pred.data.cpu().numpy().shape[0], 1))

    valid_classification_accuracy = valid_correct_class / total_size
    logging.info('Validation loss: {}. Validation classification accuracy: {}'
                 .format(np.mean(loss_history), valid_classification_accuracy))
    print('Validation loss: {}. Classification accuracy: {}.'
          .format(np.mean(loss_history), valid_classification_accuracy))

    return valid_loss
# ========================================================================


# ========================================================================
# Main function:

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='train neural network based on feature files and save parameters')
    parser.add_argument('train_file', type=str, help='training data (h5)')
    parser.add_argument('valid_file', type=str, help='validation data (h5)')
    parser.add_argument('-n', '--network-structure', type=int, default=1,
                        help='Specify which network to use:\n \
              \t 0: Fully connected neural network.\n \
              \t 1: 1D-CNN for lane feature extraction.')
    parser.add_argument('-d', '--data-loader', action='store_true',
                        help='Use the dataloader (when memory size is smaller than dataset size)')
    parser.add_argument('-s', '--save-path', type=str, default='./',
                        help='Specify the directory to save trained models.')
    parser.add_argument('-g', '--go', action='store_true',
                        help='It is training lane-follow (go) cases.')
    parser.add_argument('-b', '--balance', type=float, default=1.0,
                        help='Specify the weight for positive predictions.')
    # parser.add_argument('-g', '--gpu_num', type=int, default=0, \
    #    help='Specify which GPU to use.')

    args = parser.parse_args()

    # os.environ['CUDA_DEVICE_ORDER'] = 'PCI_BUS_ID' #specifies the same order as nvidia-smi
    #os.environ['CUDA_VISIBLE_DEVICES'] = str(args.gpu_num)

    if not args.data_loader:

        # Load from file and print out general information of the data.
        train_file = args.train_file
        valid_file = args.valid_file
        train_data = load_data(train_file)
        valid_data = load_data(valid_file)
        print('Data loaded successfully.')
        classes_train = np.asarray(train_data[:, -dim_output])
        print('Total number of training samples: {}'.format(len(classes_train)))
        print('Training set distribution:')
        print_dist(classes_train)
        classes_valid = np.asarray(valid_data[:, -dim_output])
        print('Total number of validation samples: {}'.format(len(classes_valid)))
        print('Validation set distribution:')
        print_dist(classes_valid)

        # Data preprocessing
        X_train, y_train = data_preprocessing(train_data)
        X_valid, y_valid = data_preprocessing(valid_data)

        # Model declaration
        model = None
        if args.network_structure == 0:
            model = FullyConn_NN()
        elif args.network_structure == 1:
            model = FCNN_CNN1D()
        print("The model used is: ")
        print(model)
        learning_rate = 6.561e-4
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, factor=0.3, patience=2, min_lr=1e-8, verbose=1, mode='min')

        # CUDA set-up:
        cuda_is_available = torch.cuda.is_available()
        if (cuda_is_available):
            print("Using CUDA to speed up training.")
            model.cuda()
            X_train = Variable(torch.FloatTensor(X_train).cuda())
            X_valid = Variable(torch.FloatTensor(X_valid).cuda())
            y_train = Variable(torch.FloatTensor(y_train).cuda())
            y_valid = Variable(torch.FloatTensor(y_valid).cuda())

        # Model training:
        pos_label = 1.0
        if args.go:
            pos_label = 0.0
        best_valid_loss = float('+inf')
        for epoch in range(50):
            train_vanilla(X_train, y_train, model, optimizer,
                          epoch, balance=args.balance)
            valid_loss = validate_vanilla(
                X_valid, y_valid, model, balance=args.balance, pos_label=pos_label)
            scheduler.step(valid_loss)
            if valid_loss < best_valid_loss:
                best_valid_loss = valid_loss
                torch.save(model.state_dict(), args.save_path + 'cruise_model{}_epoch{}_valloss{:.6f}.pt'
                           .format(args.network_structure, epoch+1, valid_loss))

    else:
        train_dir = args.train_file
        valid_dir = args.valid_file

        # Data preprocessing (training data balancing).
        list_of_training_files = getListOfFiles(train_dir)
        list_of_validation_files = getListOfFiles(valid_dir)

        classes_train = []
        for file in list_of_training_files:
            with h5py.File(file, 'r') as h5_file:
                data = h5_file[list(h5_file.keys())[0]][:, -2]
                classes_train.append(data.tolist())
        # "Flattening" the list of lists
        classes_train = [item for sublist in classes_train for item in sublist]
        classes_train = np.asarray(classes_train)
        print('Total number of training samples: {}'.format(len(classes_train)))
        print('Training set distribution:')
        print_dist(classes_train)

        classes_valid = []
        for file in list_of_validation_files:
            with h5py.File(file, 'r') as h5_file:
                data = h5_file[list(h5_file.keys())[0]][:, -2]
                classes_valid.append(data.tolist())
        # "Flattening" the list of lists
        classes_valid = [item for sublist in classes_valid for item in sublist]
        classes_valid = np.asarray(classes_valid)
        print('Total number of validation samples: {}'.format(len(classes_valid)))
        print('Validation set distribution:')
        print_dist(classes_valid)

        #class_weights = class_weight.compute_class_weight('balanced', np.unique(classes_train), classes_train)
        #weights = [class_weights[int(i+1)] for i in classes_train]
        #weights = torch.DoubleTensor(weights)
        #train_sampler = sampler.WeightedRandomSampler(weights, int(len(weights)/1), replacement=True)

        model = FCNN_CNN1D()
        learning_rate = 6.561e-4
        optimizer = optim.Adam(model.parameters(), lr=learning_rate)
        scheduler = optim.lr_scheduler.ReduceLROnPlateau(
            optimizer, factor=0.3, patience=2, min_lr=1e-8, verbose=1, mode='min')
        if (cuda_is_available):
            print('Using CUDA to speed up training.')
            model.cuda()

        train_dataset = TrainValidDataset(list_of_training_files)
        valid_dataset = TrainValidDataset(list_of_validation_files)

        train_loader = DataLoader(train_dataset, batch_size=1024, num_workers=8,
                                  pin_memory=True, shuffle=True)  # sampler=train_sampler)
        valid_loader = DataLoader(
            valid_dataset, batch_size=1024, num_workers=8, pin_memory=True)

        for epoch in range(100):
            train_dataloader(train_loader, model, optimizer, epoch)
            valid_loss = validate_dataloader(valid_loader, model)
            scheduler.step(valid_loss)

# ========================================================================
