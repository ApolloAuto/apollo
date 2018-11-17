# 如何在预测模块中添加新评估器

## 简介
评估器通过应用预训练的深度学习模型生成特征（来自障碍物和当前车辆的原始信息）以获得模型输出。

## 添加评估器的步骤
按照下面的步骤添加名称为`NewEvaluator`的评估器。
1. 在proto中添加一个字段
2. 声明一个从`Evaluator`类继承的类`NewEvaluator`
3. 实现类`NewEvaluator`
4. 更新预测配置
5. 更新评估器管理

### 声明一个从`Evaluator`类继承的类`NewEvaluator`
 `modules/prediction/evaluator/vehicle`目录下新建文件`new_evaluator.h`。声明如下:
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

### 实现类 `NewEvaluator`
在`new_evaluator.h`所在目录下新建文件`new_evaluator.cc`。实现如下:
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

### 在proto中添加新评估器
在`prediction_conf.proto`中添加新评估器类型:
```cpp
  enum EvaluatorType {
    MLP_EVALUATOR = 0;
    NEW_EVALUATOR = 1;
  }
```

### 更新prediction_conf文件
在 `modules/prediction/conf/prediction_conf.pb.txt`中，按照如下方式更新字段`evaluator_type`:
```
obstacle_conf {
  obstacle_type: VEHICLE
  obstacle_status: ON_LANE
  evaluator_type: NEW_EVALUATOR
  predictor_type: NEW_PREDICTOR
}
```

### 更新评估器管理
按照如下方式更新`CreateEvluator( ... )` :
```cpp
  case ObstacleConf::NEW_EVALUATOR: {
      evaluator_ptr.reset(new NewEvaluator());
      break;
    }
```
按照如下方式更新`RegisterEvaluators()` :
```cpp
  RegisterEvaluator(ObstacleConf::NEW_EVALUATOR);
```

完成上述步骤后，新评估器便创建成功了。

## 添加新特性
如果你想添加新特性，请按照如下的步骤进行操作:
### 在proto中添加一个字段
假设新的评估结果名称是`new_output`且类型是`int32`。如果输出直接与障碍物相关，可以将它添加到`modules/prediction/proto/feature.proto`中，如下所示:
```cpp
message Feature {
    // Other existing features
    optional int32 new_output = 1000;
}
```

如果输出与车道相关，请将其添加到`modules/prediction/proto/lane_graph.proto`中，如下所示:
```cpp
message LaneSequence {
    // Other existing features
    optional int32 new_output = 1000;
}
```

