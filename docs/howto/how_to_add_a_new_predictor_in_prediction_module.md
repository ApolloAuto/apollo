# How to add a new predictor in prediction module

## Introduction
Predictor generates the predicted trajectory for each obstacle. Here assume we want to add a new predictor for vehicle, for other types of obstacles, the procedure is very similar.

## Steps to add a new predictor
Please follow the steps to add a new predictor named `NewPredictor`.

### Step 1: Define a class that inherits `Predictor`
Create a new file named `new_predictor.h` in the folder  `modules/prediction/predictor/vehicle`. And define it like this:
```cpp

#include "modules/prediction/predictor/predictor.h"

namespace apollo {
namespace prediction {

class NewPredictor : public Predictor {
 public:
  void Predict(Obstacle* obstacle) override;
  // Other useful functions and fields.
};

}  // namespace prediction
}  // namespace apollo
```

### Step 2 Implement the class `NewPredictor`
Create a new file named `new_predictor.cc` in the same folder of `new_predictor.h`. Implement it like this:
```cpp
#include "modules/prediction/predictor/vehicle/new_predictor.h"

namespace apollo {
namespace prediction {

NewPredictor::Predict(Obstacle* obstacle)() {
  // Get the results from evaluator
  // Generate the predicted trajectory
}

// Other functions

}  // namespace prediction
}  // namespace apollo

```

### Step 3: Add a new predictor type in proto `prediction_conf.proto`
```
  enum PredictorType {
    LANE_SEQUENCE_PREDICTOR = 0;
    FREE_MOVE_PREDICTOR = 1;
    REGIONAL_PREDICTOR = 2;
    MOVE_SEQUENCE_PREDICTOR = 3;
    NEW_PREDICTOR = 4;
  }
```

### Step 4: Update prediction conf
In the file `modules/prediction/conf/prediction_conf.pb.txt`, update the field `predictor_type` like this:
```
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: NEW_EVALUATOR
  predictor_type: NEW_PREDICTOR
}
```

### Step 5: Upate predictor manager
Update `CreateEvluator( ... )` like this:
```cpp
  case ObstacleConf::NEW_PREDICTOR: {
      predictor_ptr.reset(new NewPredictor());
      break;
    }
```
Update `RegisterPredictors()` like this:
```cpp
  RegisterPredictor(ObstacleConf::NEW_PREDICTOR);
```
After this procedure, the new predictor will be created.
