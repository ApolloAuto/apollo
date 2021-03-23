import numpy as np

obstacle_data = [[123,1,2,4,56,7,8,8,9,9,9,6,5,5]]

with open('/apollo/auto-test/data/obstacles.csv', 'a') as csv:
    np.savetxt(csv, obstacle_data, fmt='%.4f', delimiter=',')