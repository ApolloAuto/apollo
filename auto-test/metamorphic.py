import numpy as np
from numpy import genfromtxt

# load data from data/obstacles.csv and data/collision.csv
obstacles_data = genfromtxt('/apollo/auto-test/data/obstacles.csv', delimiter=',')
collision_data = genfromtxt('/apollo/auto-test/data/collision.csv', delimiter=',')

print(obstacles_data.shape)
# remove duplicated data in collision_data
collision_data = np.unique(collision_data)
collision_size = collision_data.size
# find the information of all the collision obstacles in obstacle_data
collision_obstacle_data = np.zeros([collision_size, 10])
for c in range(collision_size):
    obstacle_info = obstacles_data[obstacles_data[:,0] == collision_data[c]]
    collision_obstacle_data[c] = obstacle_info[0]



# print collision rate
# get the number of scenarios generated
scenario_number = obstacles_data[-1][1]
general_rate = float(collision_size) / float(scenario_number)
print("Total Obstacle Generated: %d" % obstacles_data.size)
print("Total Round of Scenario: %d" % scenario_number)
print("Number of Collision Detected: %d" % collision_size)
print("General Collision Rate (#collision / #scenerio): %.4f%%" % (general_rate * 100))

collision_analysis = np.array([[obstacles_data.size, scenario_number, collision_size, general_rate]])
with open('/apollo/auto-test/data/collision_analysis.csv', 'a') as csv:
    np.savetxt(csv, collision_analysis, 
               fmt=['%d','%d','%d','%.6f'], 
               delimiter=',')

