/***************************************************************************
 *
 * Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
 *
 **************************************************************************/
/**
 * @file world_manager.h
 **/
#ifndef MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SCENARIO_MANAGER_H
#define MODULES_PLANNING_LATTICE_BEHAVIOR_DECIDER_SCENARIO_MANAGER_H

#include "modules/common/macro.h"

namespace apollo {
namespace planning {

//class ScenarioFeature;
class ScenarioManager {
private:
    enum FeatureLevel {
        LEVEL0 = 0,
        LEVEL1,
        LEVEL2,
        LEVEL3,
        NUM_LEVELS,
    };
public:
    /*
    void reset();
    int construct_world(const DecisionPath* decision_path);
    template<class T>
    void register_world_feature(FeatureLevel level) {
        auto feature = std::make_unique<T>();
        _features[static_cast<int>(level)].push_back(feature.get());
        _indexed_features[feature->name()] = std::move(feature);
    }
    template<class T>
    const T* find_world() const {
        auto world_iter = _indexed_features.find(T::feature_name());
        if (world_iter == _indexed_features.end()) {
            return nullptr;
        } else {
            return dynamic_cast<const T*>(world_iter->second.get());
        }
    }
    */
private:
    /*
    void register_world_features();
    std::vector<std::vector<WorldFeature*>> _features;
    std::unordered_map<std::string, std::unique_ptr<WorldFeature>> _indexed_features;
    bool _initialized = false;
    */
    DECLARE_SINGLETON(ScenarioManager);
};
} // namespace decision
} // namespace adu
} // namespace baidu
#endif
