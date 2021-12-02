# Jointly VectorNet-TNT-Interaction Evaluator

# Introduction

The prediction module comprises 4 main functionalities: Container, Scenario, Evaluator and Predictor.

An Evaluator predicts paths and speeds for any given obstacles. An evaluator evaluates a path(lane sequence) with a probability by the given model stored in prediction/data/.

In Apollo 7.0, a new model named Jointly VectorNet-TNT-Interaction Evaluator is
used to generate short term trajectory points. This model uses VectorNet as
encoder and TNT as decoder, in which the planning information of ADC is inserted
at the end. For scenarios at junction, the obstacle trajectory prediction by the
new model is more accurate according to the results of our experiments and road tests.

![Diagram](images/interaction_model_fig_1.png)

# Where is the code

Please refer [jointly prediction planning evaluator](https://github.com/ApolloAuto/apollo/tree/master/modules/prediction/evaluator/vehicle).

# Code Reading

## Interaction filter
Please refer [interaction filter](https://github.com/ApolloAuto/apollo/tree/master/modules/prediction/scenario/interaction_filter).
1. The interaction filter is a rule-based filter for selecting interactive obstacles.

2. Such interactive obstacles will be labeled.

    ```cpp
    void AssignInteractiveTag();
    ```

# Algorithm Detail
## Structure
![Diagram](images/VectorNet-TNT-Interaction.png)

## Encoder
Basically, the encoder is mainly using an [VectorNet](https://arxiv.org/abs/2005.04259). The ADC trajectory and all obstacle trajectories in the form of coordinate points are transformed into polylines. For each polyline, vectors are connected sequentially, where it contains start point, end point, obstacle length and some other attributes of vector. All points are transformed in the coordinate of the ADC, with North as the direction and (0, 0) as the position for ADC at time 0.

After that, map information is extracted from HDMap files. As structures of lane/road/junction/crosswalk are depicted in points in HDMap, they are also processed as polylines whose vectors contain the types of traffic elements. In order to speed up calculation in Network, only information in ROI is taken into consideration.

A Subgraph is used to deal with each polyline and the feature of polyline is acquired. After that, the polyline features are inputted into a Global Graph, which is a GNN Network substantially.

The encoding feature is gained after encoding of VectorNet. For more information in detail, please find the References.

## Decoder
The structure of Decoder is a [TNT model](https://arxiv.org/abs/2008.08294).

There are three parts in TNT.
1)Target Prediction. N points around the ADC is grid-sampled and M points are selected as target points. These target points are used as the final points of prediction trajectories in the next step.
2)Motion Estimation. With VectorNet feature and target points, a prediction trajectory is generated for each selected target point. To train more efficiently, a teaching method using true trajectory is also used in this step.
3)Scoring and Selection. A score is calculated for each motion trajectory in the second step, which is used to select final trajectories.

## Interactive Planning
After three steps in TNT, K predicted trajectories are assumed to be outputted. There is a latest planning trajectory with the same form, with which the distance of position and velocity can be calculated, named as (cost1, cost2) respectively. Meanwhile, weights of (cost1, cost2) are also outputted by VectorNet, outputting the final cost by multiplication.

Note we can also find a cost with the truth obstacle trajectory and ADC planning, thus producing the true value of cost. That's how the loss is calculated in this step.


# References
1. Gao, Jiyang, et al. "Vectornet: Encoding hd maps and agent dynamics from vectorized representation." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2020.
2.Zhao, Hang, et al. "Tnt: Target-driven trajectory prediction." arXiv preprint arXiv:2008.08294 (2020).
