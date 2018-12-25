# Open Space Planner Algorithm

## Introduction

As the Planning module introduced a major change with the launch of Scenario-based planning, we realized the need to have scenario specific algorithms that can be fine-tuned to enhance the trajectory of the ego-car for that particular scenario. The Open Space Planner is one such algorithm in development targeted towards reverse parking and sharp U-turns

## Algorithm Flow

![](images/Open_space_planner.png)

This algorithm takes in input from two different sources:
- Perception data which includes but is not limited to obstacles
- The Region of Interest (ROI) which is obtained via HD Map

Once the data is in place, Open Space planner is triggered as seen in the image above. The algorithm itself is comprised of two stages:

### Searching - Based Planning

In step 1, a raw trajectory is generated for the ego-car. This stage applies vehicle kinemetic model in algorithm to create the raw trajectory with a series of distance equidistant points as seen in the image below. 
The red line represents the raw trajectory output which is sent to the next step, Optimization to calculate the green smoothened line.

![](images/step1.png)


### Optimization


This step involves two major tasks, 
- Smooth the trajectory to get better riding comfort experience and to make it easier for the control module to track
- Ensure collision avoidance

The received raw trajectory is taken as an initial guess for optimization to iterate on. The generated result is a set of points that are not distributed evenly but are closer to eachother near the turning while those on a linear path are more spread-out.
This not only ensures better turns, but as time/space is fixed, the nearer the points, the slower the speed of the ego-car. Which also means that velocity tracking in this step is possible but more reasonable acceleration, braking and steering.

![](images/step2.png)

Once this stage is complete, the output is directly sent to the Control module to have it sent to the ego-car.

![](images/step3.png)

## Future Applications

As the algorithm is currently in development, it is currently used for reverse parking, but can also be implemented in scenarios that involve tight U-turns. The main aim of scenario-based planning is to ensure efficient planning of the car's trajectory using targeted algorithms for individual scenarios just like with Open Space Planning for reverse parking.
