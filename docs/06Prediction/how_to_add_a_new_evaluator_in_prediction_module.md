# How to add a new evaluator in Prediction module

## Introduction
The Evaluator generates features (from the raw information of obstacles and the ego vehicle) to get the model output by applying the pre-trained deep learning model.

## Steps to add a new evaluator
Please follow the steps to add a new evaluator named `NewEvaluator`.
1. Add a field in proto
2. Define a class that inherits `Evaluator`
3. Implement the class `NewEvaluator`
4. Update prediction conf
5. Upate the evaluator manager

### Define a class that inherits `Evaluator`
Create a new file named `new_evaluator.h` in the folder  `modules/prediction/evaluator/vehicle`. And define it like this:
```cpp
#include "modules/prediction/evaluator/evaluator.h"

namespace apollo {
namespace prediction {

class NewEvaluator : public Evaluator {
 public:
  NewEvaluator();
  virtual ~NewEvaluator();
  void Evaluate(Obstacle* obstacle_ptr) override;
  // Other useful functions and fields.
};

}  // namespace prediction
}  // namespace apollo
```

### Implement the class `NewEvaluator`
Create a new file named `new_evaluator.cc` in the same folder as that of `new_evaluator.h`. Implement it like this:
```cpp
#include "modules/prediction/evaluator/vehicle/new_evaluator.h"

namespace apollo {
namespace prediction {

NewEvaluator::NewEvaluator() {
  // Implement
}

NewEvaluator::～NewEvaluator() {
  // Implement
}

NewEvaluator::Evaluate(Obstacle* obstacle_ptr)() {
  // Extract features
  // Compute new_output by applying pre-trained model
}

// Other functions

}  // namespace prediction
}  // namespace apollo

```

### Add a new evaluator in proto
Add a new type of evaluator in `prediction_conf.proto`:
```cpp
  enum EvaluatorType {
    MLP_EVALUATOR = 0;
    NEW_EVALUATOR = 1;
  }
```

### Update prediction_conf file
In the file `modules/prediction/conf/prediction_conf.pb.txt`, update the field `evaluator_type` like this:
```
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: NEW_EVALUATOR
  predictor_type: NEW_PREDICTOR
}
```

### Step 5: Upate the evaluator manager
Update `CreateEvluator( ... )` like this:
```cpp
  case ObstacleConf::NEW_EVALUATOR: {
      evaluator_ptr.reset(new NewEvaluator());
      break;
    }
```
Update `RegisterEvaluators()` like this:
```cpp
  RegisterEvaluator(ObstacleConf::NEW_EVALUATOR);
```

After following the steps above, the new evaluator should be created.

## Adding new features
If you would like to add new features, follow the instructions that follow:
### Add a field in proto
Assume the new evaluating result named `new_output` and also assume its type is `int32`. If the output is related directly to the obstacles, you can add it into `modules/prediction/proto/feature.proto` like this:
```cpp
message Feature {
    // Other existing features
    optional int32 new_output = 1000;
}
```

If the output is related to the lane sequences, please add it into `modules/prediction/proto/lane_graph.proto` like this:
```cpp
message LaneSequence {
    // Other existing features
    optional int32 new_output = 1000;
}
```

