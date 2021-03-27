import numpy as np
from numpy import genfromtxt

# load data from data/obstacles.csv and data/collision.csv
obstacles_data = genfromtxt('/apollo/auto-test/data/obstacles.csv', delimiter=',')
collision_data = genfromtxt('/apollo/auto-test/data/collision.csv', delimiter=',')

# remove duplicated data in collision_data
collision_data = np.unique(collision_data)
collision_size = collision_data.size
# find the information of all the collision obstacles in obstacle_data
collision_obstacle_data = np.zeros([collision_size, 10])
for c in range(collision_size):
    obstacle_info = obstacles_data[obstacles_data[:,0] == collision_data[c]]
    collision_obstacle_data[c] = obstacle_info[0]

print collision_data
print collision_obstacle_data