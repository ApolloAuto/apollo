# How to add a new Predictor in Prediction module

## Introduction
The Predictor generates the predicted trajectory for each obstacle. Here, let's assume we want to add a new predictor to our vehicle, for other types of obstacles, the procedure is very as follows:
1. Define a class that inherits `Predictor`
2. Implement the class `NewPredictor`
3. Add a new predictor type in proto `prediction_conf.proto`
4. Update prediction_conf
5. Upate the Predictor manager

## Steps to add a new predictor
The following steps will add a Predictor `NewPredictor`.

### Define a class that inherits `Predictor`
Create a new file named `new_predictor.h` in the folder  `modules/prediction/predictor/vehicle` and define it as follows:
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

### Implement the class `NewPredictor`
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

### Add a new predictor type in proto `prediction_conf.proto`
```
  enum PredictorType {
    LANE_SEQUENCE_PREDICTOR = 0;
    FREE_MOVE_PREDICTOR = 1;
    REGIONAL_PREDICTOR = 2;
    MOVE_SEQUENCE_PREDICTOR = 3;
    NEW_PREDICTOR = 4;
  }
```

### Update prediction_conf
In the file `modules/prediction/conf/prediction_conf.pb.txt`, update the field `predictor_type` like this:
```
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: NEW_EVALUATOR
  predictor_type: NEW_PREDICTOR
}
```

### Upate Predictor manager
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

After completing the steps above, you would have created a new Predictor.