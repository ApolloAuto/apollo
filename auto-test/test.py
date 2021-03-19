import numpy as np

for i in range(10):

    if (np.random.rand() > .5):
        print(i, 'high')
    else:
        print(i, 'low')