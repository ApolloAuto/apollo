#!/usr/bin/env bash

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

declare -A dic
models_dir="/apollo/modules/perception/data/models/"

dic=([3d-r4-half_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/3d-r4-half_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T09%3A39%3A46Z/-1/host/35f44fc343144e9a6f2d9f19a1daef32ab32bdbdb21f9bfab13ba235e3118c97" \
     [cnnseg128_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/cnnseg128_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T10%3A49%3A05Z/-1/host/17850b8a5f73333291bbdedcb0103222a70b143516d86f8af79649f4f4100543" \
     [cnnseg64_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/cnnseg64_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T10%3A49%3A21Z/-1/host/390c40b2946bfba35910c6c3de738eaafdc1b11b15c9131c4a437f72a026deb4" \
     [cnnseg16_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/cnnseg16_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T10%3A49%3A40Z/-1/host/a8996774dd80355a16e47949d1488b785faf8d2093252d2ccd41263a0a74a44b" \
     [horizontal_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/horizontal_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T10%3A51%3A41Z/-1/host/c1c84568d35ac87f99f4a27a00baf00e0a19a5034be32a02b0e4cd584ace4ef0" \
     [quadrate_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/quadrate_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T11%3A03%3A53Z/-1/host/7aa8d34acdc71de4acae2fa7ee46267b7e504751f36c0fcff8978d292322bde3" \
     [vertical_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/vertical_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T11%3A04%3A55Z/-1/host/6762735da1dbd5095efc72b3e32ee46d9d25671c3bc68dc8930c9f10eed865d9" \
     [tl_detection_caffe.zip]="https://apollo-perception.bj.bcebos.com/core_model/tl_detection_caffe.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T11%3A04%3A40Z/-1/host/f00446d0d66465d7aa55723605a29c6061f1ff7dec34e7d926b6f5d63592106b" \
     [smoke_torch.zip]="https://apollo-perception.bj.bcebos.com/core_model/smoke_torch.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-27T11%3A04%3A22Z/-1/host/b9aed27fd7480add4225a7fe8f83b3f81aecde39035b057837ef3ff9ccc93055" \
     [yolox3d.zip]="https://apollo-perception.bj.bcebos.com/core_model/yolox3d.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-31T09%3A28%3A13Z/-1/host/344d35f81d3b12a8f772526087b2a5989bf3b8594bf1d9a125ef1e01020177e7" \
     [center_point_paddle.zip]="https://apollo-perception.bj.bcebos.com/core_model/center_point_paddle.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-10-31T09%3A29%3A10Z/-1/host/8f45ff099aa79c6043c744cf9ff72fb432335ac60aa3fe9235b9ec275e003aab" \
     [point_pillars.zip]="https://apollo-perception.bj.bcebos.com/core_model/point_pillars.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-11-01T05%3A14%3A59Z/-1/host/595359299542b1dbf657d5515996f29b909898a31c0b6708fc309d4d8d37735a" \
     [point_pillars_torch.zip]="https://apollo-perception.bj.bcebos.com/core_model/point_pillars_torch.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-11-02T09%3A28%3A04Z/-1/host/8c5356cf4f38c3590017fe552acb9764dac8696f92324aee414ce7eaa029b57a" \
     [mask_pillars_torch.zip]="https://apollo-perception.bj.bcebos.com/core_model/mask_pillars_torch.zip?authorization=bce-auth-v1/e3384375161a482d8fa77e1ef2d32e05/2023-11-02T09%3A28%3A22Z/-1/host/129833d8aacecc3a95e729f73d73d6b44bfcb5c8a23eb1e5051b71b4b9f8a075" \
    )

cd "/apollo"

for key in $(echo ${!dic[*]}); do
    download_link=${dic[$key]}
    wget -O ${key} ${download_link}
    unzip ${key}
    for i in `ls ${key::-4}`; do
        [[ -e ${models_dir}${key::-4}/${i} ]] && rm -f ${models_dir}${key::-4}/${i}
        sudo bash -c "mv -f ${key::-4}/${i} ${models_dir}${key::-4}/"
    done
    rm -rf ${key::-4} ${key}
  done
