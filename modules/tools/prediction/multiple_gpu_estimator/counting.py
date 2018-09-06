import numpy as np
import os
from collections import Counter
from glob import glob

feature_dim = 62
count = Counter()
filenames = glob('/tmp/data/feature_v1_bin/*/*.label.bin')
for filename in filenames:
    bin_data = np.fromfile(filename, dtype=np.float32)
    if bin_data.shape[0] % (feature_dim + 1) != 0:
        raise ValueError('data size (%d) must be multiple of feature_dim + 1 (%d).' %(bin_data.shape[0], feature_dim + 1))
    label = bin_data[feature_dim::(feature_dim+1)].astype(np.int32)
    count.update(label)

print(count)