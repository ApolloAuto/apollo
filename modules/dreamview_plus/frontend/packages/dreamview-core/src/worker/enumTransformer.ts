// todo: 暂时废弃，后续需要根据实际情况进行调整
import ValueReplacer, { type ValueTransformer } from '../util/ValueReplacer';
import {
    ChangeLaneType,
    ComponentStatus,
    DisengageType,
    GearPosition,
    HMIModeOperation,
    InteractiveTag,
    LoadRecordStatus,
    LogLevel,
    MessageSource,
    ObjectType,
    PlayRecordStatus,
    Priority,
    Source,
    StopReasonCode,
    SubType,
    V2XType,
} from './enum.type';

export function enumTransformer(enumType: any): ValueTransformer {
    const numToName = (value: number | string) => {
        const num = typeof value === 'string' ? parseInt(value, 10) : value;
        return enumType[num] || `Unknown value: ${num}`;
    };

    const nameToNum = (name: string) => {
        const key = Object.keys(enumType).find((k) => enumType[k] === name);
        return key ? enumType[key] : `Unknown name: ${name}`;
    };

    return (value: any): any => {
        if (Array.isArray(value)) {
            // 处理数组，每个元素都转换
            return value.map((val) => {
                if (typeof val === 'number' || (typeof val === 'string' && !Number.isNaN(parseInt(val, 10)))) {
                    return numToName(val);
                }
                return nameToNum(val);
            });
        }
        if (typeof value === 'number' || (typeof value === 'string' && !Number.isNaN(parseInt(value, 10)))) {
            return numToName(value);
        }
        if (typeof value === 'string') {
            return nameToNum(value);
        }
        return `Unsupported type: ${typeof value}`;
    };
}

const replacer = new ValueReplacer();

replacer.registerTransformer(
    'apollo.dreamview.HMIStatus',
    ['$.operations', '$.currentOperation'],
    enumTransformer(HMIModeOperation),
);

// 注册转换器
replacer.registerTransformer(
    'apollo.dreamview.HMIStatus',
    '$.records[*].loadRecordStatus',
    enumTransformer(LoadRecordStatus),
);

replacer.registerTransformer(
    'apollo.dreamview.HMIStatus',
    'currentRecordStatus.playRecordStatus',
    enumTransformer(PlayRecordStatus),
);

replacer.registerTransformer(
    'apollo.dreamview.HMIStatus',
    'monitoredComponents.*.status',
    enumTransformer(ComponentStatus),
);

replacer.registerTransformer('apollo.dreamview.Obstacles', '$..decision[*].type', enumTransformer(ComponentStatus));

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    '$..decision[*].type',
    enumTransformer(ComponentStatus),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    '$..decision[*].stopReason',
    enumTransformer(StopReasonCode),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    '$..decision[*].changeLaneType',
    enumTransformer(ChangeLaneType),
);

replacer.registerTransformer('apollo.dreamview.SimulationWorld', '$..gearLocation', enumTransformer(GearPosition));

replacer.registerTransformer('apollo.dreamview.SimulationWorld', '$..disengageType', enumTransformer(DisengageType));

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    [
        '$.sensorMeasurements..sensorMeasurement[*].subType',
        '$.object[*].subType',
        '$.autoDrivingCar.subType',
        '$.trafficSignal.subType',
        '$.planningTrajectory[*].subType',
        '$.mainStop.subType',
        '$.mainDecision.subType',
        '$.gps.subType',
        '$.shadowLocalization.subType',
        '$.perceivedSignal[*].subType',
    ],
    enumTransformer(SubType),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    [
        '$.sensorMeasurements..sensorMeasurement[*].type',
        '$.object[*].type',
        '$.autoDrivingCar.type',
        '$.trafficSignal.type',
        '$.planningTrajectory[*].type',
        '$.mainStop.type',
        '$.mainDecision.type',
        '$.gps.type',
        '$.shadowLocalization.type',
        '$.perceivedSignal[*].type',
    ],
    enumTransformer(ObjectType),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    [
        '$.sensorMeasurements..sensorMeasurement[*].obstaclePriority.priority',
        '$.object[*].obstaclePriority.priority',
        '$.autoDrivingCar.obstaclePriority.priority',
        '$.trafficSignal.obstaclePriority.priority',
        '$.planningTrajectory[*].obstaclePriority.priority',
        '$.mainStop.obstaclePriority.priority',
        '$.mainDecision.obstaclePriority.priority',
        '$.gps.obstaclePriority.priority',
        '$.shadowLocalization.obstaclePriority.priority',
        '$.perceivedSignal[*].obstaclePriority.priority',
    ],
    enumTransformer(Priority),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    [
        '$.sensorMeasurements..sensorMeasurement[*].interactiveTag.interactiveTag',
        '$.object[*].interactiveTag.interactiveTag',
        '$.autoDrivingCar.interactiveTag.interactiveTag',
        '$.trafficSignal.interactiveTag.interactiveTag',
        '$.planningTrajectory[*].interactiveTag.interactiveTag',
        '$.mainStop.interactiveTag.interactiveTag',
        '$.mainDecision.interactiveTag.interactiveTag',
        '$.gps.interactiveTag.interactiveTag',
        '$.shadowLocalization.interactiveTag.interactiveTag',
        '$.perceivedSignal[*].interactiveTag.interactiveTag',
    ],
    enumTransformer(InteractiveTag),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    [
        '$.sensorMeasurements..sensorMeasurement[*].source',
        '$.object[*].source',
        '$.autoDrivingCar.source',
        '$.trafficSignal.source',
        '$.planningTrajectory[*].source',
        '$.mainStop.source',
        '$.mainDecision.source',
        '$.gps.source',
        '$.shadowLocalization.source',
        '$.perceivedSignal[*].source',
    ],
    enumTransformer(Source),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    [
        '$.sensorMeasurements..sensorMeasurement[*].v2xInfo.v2xType',
        '$.object[*].v2xInfo.v2xType',
        '$.autoDrivingCar.v2xInfo.v2xType',
        '$.trafficSignal.v2xInfo.v2xType',
        '$.planningTrajectory[*].v2xInfo.v2xType',
        '$.mainStop.v2xInfo.v2xType',
        '$.mainDecision.v2xInfo.v2xType',
        '$.gps.v2xInfo.v2xType',
        '$.shadowLocalization.v2xInfo.v2xType',
        '$.perceivedSignal[*].v2xInfo.v2xType',
    ],
    enumTransformer(V2XType),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    ['$.notification[*].item.source', '$.monitor.item[*].source'],
    enumTransformer(MessageSource),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    ['$.notification[*].item.logLevel', '$.monitor.item[*].logLevel'],
    enumTransformer(LogLevel),
);

replacer.registerTransformer(
    'apollo.dreamview.SimulationWorld',
    '$.laneMarker..laneType..types',
    enumTransformer(LogLevel),
);

export default { replacer };
