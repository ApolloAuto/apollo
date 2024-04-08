# Prediction

## Introduction

The Prediction module studies and predicts the behavior of all the obstacles detected by the perception module.
Prediction receives obstacle data along with basic perception information including positions, headings, velocities, accelerations, and then generates predicted trajectories with probabilities for those obstacles. The prediction module is composed of four sub-modules: **Container**, **Scenario**, **Evaluator** and **Predictor**.

```
Note:
The Prediction module only predicts the behavior of obstacles and not the EGO car. The Planning module plans the trajectory of the EGO car.
```

### Container

Container stores input data from subscribed channels. Current supported
inputs are **_perception obstacles_**, **_vehicle localization_** and **_vehicle planning_**.

### Scenario

The Scenario sub-module analyzes scenarios that includes the ego vehicle.
Currently, we have two defined scenarios:

- **Cruise** : this scenario includes Lane keeping and following
- **Junction** : this scenario involves junctions. Junctions can either have traffic lights and/or STOP signs.

we also have three defined types of obstacle priority:

- **Ignore**: these obstacles will not affect the ego car's trajectory and can be safely ignored (E.g. the obstacle is too far away)
- **Caution**: these obstacles have a high possibility of interacting with the ego car
- **Normal**: the obstacles that do not fall under ignore or caution are placed by default under normal

### Evaluator

The Evaluator predicts path and speed separately for any given obstacle.
An evaluator evaluates a path by outputting a probability for it (lane
sequence) using the given model stored in _prediction/data/_.

The list of available evaluators include:

- **Cost evaluator**: probability is calculated by a set of cost functions

- **MLP evaluator**: probability is calculated using an MLP model

- **RNN evaluator**: probability is calculated using an RNN model

- **Cruise MLP + CNN-1d evaluator**: probability is calculated using a mix of MLP and CNN-1d models for the cruise scenario

- **Junction MLP evaluator**: probability is calculated using an MLP model for junction scenario

- **Junction Map evaluator**: probability is calculated using an semantic map-based CNN model for junction scenario. This evaluator was created for caution level obstacles

- **Social Interaction evaluator**: this model is used for pedestrians, for short term trajectory prediction. It uses social LSTM. This evaluator was created for caution level obstacles

- **Semantic LSTM evaluator**: this evaluator is used in the new Caution Obstacle model to generate short term trajectory points which are calculated using CNN and LSTM. Both vehicles and pedestrians are using this same model, but with different parameters

- **Vectornet LSTM evaluator**: this evaluator is used in place of Semantic LSTM evaluator to generate short term trajectory points for "Caution" tagged obstacles.

- **Jointly prediction planning evaluator**: this evaluator is used in the new Interactive Obstacle(vehicle-type) model to generate short term trajectory points which are calculated using Vectornet and LSTM. By considering ADC's trajectory info, the obstacle trajectory prediction can be more accurate under interaction scenario. Please refer [jointly prediction planning evaluator](https://github.com/ApolloAuto/apollo/blob/master/docs/07_Prediction/jointly_prediction_planning_evaluator.md).

Predictor generates predicted trajectories for obstacles. Currently, the supported predictors include:

- **Empty**: obstacles have no predicted trajectories
- **Single lane**: Obstacles move along a single lane in highway navigation mode. Obstacles not on lane will be ignored.
- **Lane sequence**: obstacle moves along the lanes
- **Move sequence**: obstacle moves along the lanes by following its kinetic pattern
- **Free movement**: obstacle moves freely
- **Regional movement**: obstacle moves in a possible region
- **Junction**: Obstacles move toward junction exits with high probabilities
- **Interaction predictor**: compute the likelihood to create posterior prediction results after all evaluators have run. This predictor was created for caution level obstacles
- **Extrapolation predictor**: extends the Semantic LSTM evaluator's results to create an 8 sec trajectory.

## Structure

```
├── prediction
    ├── common                  // common code
    ├── conf                    // configuration folder
    ├── container               // container sub-module
    │   ├── adc_trajectory
    │   ├── obstacles
    │   ├── pose
    │   └── storytelling
    ├── dag                     // module startup file
    ├── data                    // module configuration parameters
    ├── evaluator               // evaluator sub-module
    │   ├── cyclist
    │   ├── model_manager
    │   ├── pedestrian
    │   ├── vehicle
    │   └── warm_up
    ├── images                  // demo images
    ├── launch                  // launch file
    ├── network                 // network code
    ├── pipeline                // VectorNet code
    ├── predictor               // predictor sub-module
    │   ├── empty
    │   ├── extrapolation
    │   ├── free_move
    │   ├── interaction
    │   ├── junction
    │   ├── lane_sequence
    │   ├── move_sequence
    │   ├── sequence
    │   └── single_lane
    ├── proto                   // configuration proto file
    ├── scenario                // scenario sub-module
    │   ├── analyzer
    │   ├── feature_extractor
    │   ├── interaction_filter
    │   ├── prioritization
    │   ├── right_of_way
    │   └── scenario_features
    ├── submodules              // manage evaluator and predictor submodules
    ├── testdata                // test data
    ├── BUILD                   // compile file
    ├── cyberfile.xml           // package management file
    ├── prediction_component.cc //component entrance
    ├── prediction_component.h
    └── prediction_component_test.cc

```

## Modules

### PredictionComponent

#### Input

| Name    | Type                                      | Description      | Input channal |
| ------- | ----------------------------------------- | ---------------- | ------------- |
| `frame` | `apollo::perception::PerceptionObstacles` | Obstacle message | /apollo/perception/obstacles |

#### Output

| Name    | Type                                      | Description                 | Output channal |
| ------- | ----------------------------------------- | --------------------------- | -------------- |
| `frame` | `apollo::prediction::PredictionObstacles` | Obstacle prediction message | /apollo/prediction |

#### How to use

1. Modify the `modules/prediction/dag/prediction.dag`

- config_file_path: path of config file
- flag_file_path: path of flag file
- reader channel: the name of input channel

2. Modify the `modules/prediction/conf/prediction_conf.pb.txt`

- topic_conf: the name of different topic
  - xxx_topic_name： the name of xxx topic
- evaluator_model_conf: the conf of different evaluator model

  - evaluator_type: SEMANTIC_LSTM_EVALUATOR
  - obstacle_type: PEDESTRIAN, VEHICLE
  - backend: GPU, CPU
  - priority: priority of model
  - type: SemanticLstmPedestrianGpuTorch, SemanticLstmVehicleGpuTorch ...

- obstacle_conf: the conf of different obstacle
  - obstacle_type: VEHICLE, PEDESTRIAN, BICYCLE, UNKNOWN
  - obstacle_status: ON_LANE, OFF_LANE, IN_JUNCTION, MOVING
  - interactive_tag: INTERACTION
  - priority_type: CAUTION, NORMAL,IGNORE
  - evaluator_type: VECTORNET_EVALUATOR, CRUISE_MLP_EVALUATOR ...
  - predictor_type: EXTRAPOLATION_PREDICTOR, MOVE_SEQUENCE_PREDICTOR ...

3. start the prediction component

```bash
cyber_launch start modules/prediction/launch/prediction.launch
```

## Reference

1. [Xu K, Xiao X, Miao J, Luo Q. "Data Driven Prediction Architecture for Autonomous Driving and its Application on Apollo Platform." _arXiv preprint arXiv:2006.06715._ ](https://arxiv.org/pdf/2006.06715.pdf)
