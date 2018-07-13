# 如何在预测模块中添加一个预测器

## 简介

预测器为每个障碍物生成预测轨迹。在这里，假设我们想给我们的车辆增加一个新的预测器，用于其他类型的障碍，步骤如下：

1. 定义一个继承基类 `Predictor` 的类
2. 实现新类 `NewPredictor`
3. 在 `prediction_conf.proto`中添加一个新的预测期类型
4. 更新 prediction_conf
5. 更新预测器管理器（Predictor manager）

## 添加新预测器的步骤

如下步骤将会指导您在预测器中添加一个 `NewPredictor`。

### 定义一个继承基类 `Predictor` 的类

在文件夹 `modules/prediction/predictor/vehicle`中创建一个名为`new_predictor.h`的文件，文件内容如下：
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
在创建了 `new_predictor.h`的文件夹中创建文件 `new_predictor.cc`。 文件内容如下:
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

### 在 `prediction_conf.proto`中添加一个新的预测期类型
```
  enum PredictorType {
    LANE_SEQUENCE_PREDICTOR = 0;
    FREE_MOVE_PREDICTOR = 1;
    REGIONAL_PREDICTOR = 2;
    MOVE_SEQUENCE_PREDICTOR = 3;
    NEW_PREDICTOR = 4;
  }
```

### 更新 prediction_conf
在 `modules/prediction/conf/prediction_conf.pb.txt`中, 更新 `predictor_type`部分如下:
```
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: NEW_EVALUATOR
  predictor_type: NEW_PREDICTOR
}
```

### 更新预测器管理器（Predictor manager）
更新 `CreateEvluator( ... )` 如下:
```cpp
  case ObstacleConf::NEW_PREDICTOR: {
      predictor_ptr.reset(new NewPredictor());
      break;
    }
```
更新 `RegisterPredictors()` 如下:
```cpp
  RegisterPredictor(ObstacleConf::NEW_PREDICTOR);
```

在完成以上步骤以后，一个新的预测器就创建好了。
=======
# 如何在预测模块中添加一个预测器

## 简介

预测器为每个障碍物生成预测轨迹。在这里，假设我们想给我们的车辆增加一个新的预测器，用于其他类型的障碍，步骤如下：

1. 定义一个继承基类 `Predictor` 的类
2. 实现新类 `NewPredictor`
3. 在 `prediction_conf.proto`中添加一个新的预测期类型
4. 更新 prediction_conf
5. 更新预测器管理器（Predictor manager）

## 添加新预测器的步骤

如下步骤将会指导您在预测器中添加一个 `NewPredictor`。

### 定义一个继承自基类 `Predictor` 的类

在文件夹 `modules/prediction/predictor/vehicle`中创建一个名为`new_predictor.h`的文件，文件内容如下：
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
在创建了 `new_predictor.h`的文件夹中创建文件 `new_predictor.cc`。文件内容如下:
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

### 在 `prediction_conf.proto`中添加一个新的预测器类型
```
  enum PredictorType {
    LANE_SEQUENCE_PREDICTOR = 0;
    FREE_MOVE_PREDICTOR = 1;
    REGIONAL_PREDICTOR = 2;
    MOVE_SEQUENCE_PREDICTOR = 3;
    NEW_PREDICTOR = 4;
  }
```

### 更新 prediction_conf
在 `modules/prediction/conf/prediction_conf.pb.txt`中，更新 `predictor_type`部分如下:
```
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: NEW_EVALUATOR
  predictor_type: NEW_PREDICTOR
}
```

### 更新预测器管理器（Predictor manager）
更新 `CreateEvluator( ... )` 如下:
```cpp
  case ObstacleConf::NEW_PREDICTOR: {
      predictor_ptr.reset(new NewPredictor());
      break;
    }
```
更新 `RegisterPredictors()` 如下:
```cpp
  RegisterPredictor(ObstacleConf::NEW_PREDICTOR);
```

完成以上步骤后，一个新的预测器就创建好了。
