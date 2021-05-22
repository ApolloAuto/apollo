# PREDICTION PREDICTOR

# Introduction

The prediction module comprises 4 main functionalities: Container, Scenario, Evaluator and Predictor. 

Predictor generates predicted trajectories for obstacles. Currently, the supported predictors include:

- Empty: obstacles have no predicted trajectories
- Single lane: Obstacles move along a single lane in highway navigation mode. Obstacles not on lane will be ignored.
- Lane sequence: obstacle moves along the lanes
- Move sequence: obstacle moves along the lanes by following its kinetic pattern
- Free movement: obstacle moves freely
- Regional movement: obstacle moves in a possible region
- Junction: Obstacles move toward junction exits with high probabilities
- Interaction predictor: compute the likelihood to create posterior prediction results after all evaluators have run. This predictor was created for caution level obstacles
- Extrapolation predictor: extends the Semantic LSTM evaluator's results to create an 8 sec trajectory.

Here, we mainly introduce three typical predictors，extrapolation predictor, move sequence predictor and interaction predictor，other predictors are similar to them.  

# Where is the code

Please refer [prediction predictor](https://github.com/ApolloAuto/apollo/modules/prediction/predictor).

# Code Reading

## Extrapolation predictor
1. This predictor is used to extend the Semantic LSTM evaluator's results to creat a long-term trajectroy(which is 8 sec).

2. There are two main kinds of extrapolation, extrapolate by lane and extrapolate by free move.
     1. Base on a search radium and an angle threshold, which can be changed in perdiction config, we get most likely lane that best matches the short-term predicted trajectory obtained from Semantic LSTM evaluator.
     ```cpp
            LaneSearchResult SearchExtrapolationLane(const Trajectory& trajectory,
                                                     const int num_tail_point);
     ```
     2. If the matching lane is found, we extend short-term predicted trajectory by lane.
         1. Firstly, we remove points those are not in the matching lane.
         2. Then, we project the modified short-term predicted trajectory onto the matching lane to get SL info.
         ```cpp
                static bool GetProjection(
                const Eigen::Vector2d& position,
                const std::shared_ptr<const hdmap::LaneInfo> lane_info, double* s,double* l);
         ```
         3. According to prediction horizon, we extend the modified short-term predicted trajectory along the mathcing lane with a constant-velocity module and get smooth points from lane.
         ```cpp
                static bool SmoothPointFromLane(const std::string& id, const double s,
                                                const double l, Eigen::Vector2d* point,
                                                double* heading);
         ```
         4. Note that the extraplation speed, which is used in constant-velocity module, is calculated by calling ```ComputeExtraplationSpeed```.
         ```cpp 
                void ExtrapolateByLane(const LaneSearchResult& lane_search_result,
                                       const double extrapolation_speed,
                                       Trajectory* trajectory_ptr,
                                       ObstacleClusters* clusters_ptr);
         ```
         ```cpp 
                double ComputeExtraplationSpeed(const int num_tail_point,
                                                const Trajectory& trajectory);                    
         ```
3. Otherwise we use free move module to extend.
  ```cpp
    void ExtrapolateByFreeMove(const int num_tail_point,
                               const double extrapolation_speed,
                               Trajectory* trajectory_ptr);
  ```
## Move sequence predictor
1. Obstacle moves along the lanes by its kinetic pattern.

2. Ingore those lane sequences with lower probability.
 ```cpp  
    void FilterLaneSequences(
        const Feature& feature, const std::string& lane_id,
        const Obstacle* ego_vehicle_ptr,
        const ADCTrajectoryContainer* adc_trajectory_container,
        std::vector<bool>* enable_lane_sequence);  
 ```
3. If there is a stop sign on the lane, we check whether ADC supposed to stop.
     ```cpp
    bool SupposedToStop(const Feature& feature, const double stop_distance,
                        double* acceleration); 
     ```
     1. If ADC is about to stop, we produce trajectroy with constant-acceleration module.
      ```cpp
        void DrawConstantAccelerationTrajectory(
            const Obstacle& obstacle, const LaneSequence& lane_sequence,
            const double total_time, const double period, const double acceleration,
            std::vector<apollo::common::TrajectoryPoint>* points);
      ```
     2. Otherwise we produce trajectory by obstacle's kinetic pattern.
      ```cpp
        bool DrawMoveSequenceTrajectoryPoints(
            const Obstacle& obstacle, const LaneSequence& lane_sequence,
            const double total_time, const double period,
            std::vector<apollo::common::TrajectoryPoint>* points);  
     ```
## Interaction predictor
1. Compute the likelihood to create posterier prediction results after all evaluators have run. This predictor was created for caution level obstacles.

2. Sampling ADC trajectory at a fixed interval(which can be changed in prediction gflag file).
 ```cpp
    void BuildADCTrajectory(
      const ADCTrajectoryContainer* adc_trajectory_container,
      const double time_resolution);
 ```
3. Compute trajectory cost for each short-term predicted trajectory. The trajectory cost is weighted cost from different trajectory evluation metrics, such as acceleration, centripetal acceleration and collsion cost, which can be written in the following form: 
 ```
    total_cost = w_acc * cost_acc + w_centri * cost_centri + w_collision * cost_collision
 ```
Note that, the collsion cost is calucalated by the distance between ADC and obstacles.
 ```cpp
    double ComputeTrajectoryCost(
        const Obstacle& obstacle, const LaneSequence& lane_sequence,
        const double acceleration,
        const ADCTrajectoryContainer* adc_trajectory_container);
 ```
4. We use the following equration to compute the likelihood for each short-term predicted trajectory.

 ```
    likelihood = exp (-alpha * total_cost), the alpha can be changed in prediction gflag file.
 ```
 ```cpp
    double ComputeLikelihood(const double cost);
 ```
5. Base on the likelihood, we get the posterier prediction results.
 ```cpp
    double ComputePosterior(const double prior, const double likelihood);
 ```

