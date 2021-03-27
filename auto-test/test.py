import numpy as np

array = np.array([(1,0,0,0),
                  (1,2,3,4),
                  (1,1,1,1),
                  (0,1,2,3)])

print(array[array[:,1] == 1])