/* eslint-disable no-param-reassign */
import cloneDeep from 'lodash/cloneDeep';
import * as TYPES from './actionTypes';
import { HMIModeOperation } from './actionTypes';

export const TELEOP_MODE = [TYPES.CURRENT_MODE.CAR, TYPES.CURRENT_MODE.CONSOLE];

export const hmiUtils = {
    isPlayerControlShow(hmi: TYPES.IInitState) {
        return hmi.currentOperation === HMIModeOperation.PLAY_RECORDER;
    },
    isComponentsOk(components?: TYPES.COMPONENTS) {
        return components?.status === TYPES.COMPONENT_STATUS.OK;
    },
    isCalibrationMode(hmi: TYPES.IInitState) {
        return hmi.isSensorCalibrationMode || hmi.isVehicleCalibrationMode;
    },
    preConditionModule(hmi: TYPES.IInitState) {
        return hmiUtils.isCalibrationMode(hmi) ? 'Recorder' : 'none';
    },
    allMonitoredComponentSuccess(hmi: TYPES.IInitState) {
        return (
            hmiUtils.isCalibrationMode(hmi) &&
            Array.from(hmi.componentStatus.keys()).every(
                (key) => key === 'Recorder' || hmiUtils.isComponentsOk(hmi.componentStatus.get(key)),
            )
        );
    },
};

function isEqualDeep(prev: any, next: any) {
    const type = typeof prev;
    if (Array.isArray(prev) && Array.isArray(next)) {
        return prev.sort().toString() === next.sort().toString();
    }
    if (type === 'object' && prev !== null && next !== null) {
        return JSON.stringify(prev) === JSON.stringify(next);
    }
    return prev === next;
}

export const reducerHander = {
    changeVehicle(vehicle: string) {
        console.log('vehicle', vehicle);
        // fixme: 调用改变vehicle的接口
    },
    changeMap(map: string) {
        console.log('map', map);
    },
    changeScenarios(scenario: string) {
        console.log('scenario', scenario);
    },
    changeRecorder(recorderId: string) {
        console.log('recorderId', recorderId);
    },
    changeOperate(operate: string) {
        console.log('operate', operate);
    },
    changeMode(mode: string) {
        console.log('mode', mode);
    },
    toggleModule(hmi: TYPES.IInitState, prop: { key: string }) {
        console.log('module', prop);
    },
    updateStatus: (originHmi: TYPES.IInitState, draftHmi: TYPES.IInitState, newStatus: TYPES.SIM_WORLD_STATUS) => {
        const prevStatus = cloneDeep(originHmi.prevStatus);

        if (!isEqualDeep(prevStatus.dockerImage, newStatus.dockerImage)) {
            draftHmi.dockerImage = newStatus.dockerImage;
        }

        if (!isEqualDeep(prevStatus.modes, newStatus.modes)) {
            draftHmi.modes = newStatus.modes.sort();
        }

        if (!isEqualDeep(prevStatus.currentMode, newStatus.currentMode)) {
            draftHmi.isVehicleCalibrationMode = newStatus.currentMode.toLowerCase().includes('vehicle calibration');
            draftHmi.isSensorCalibrationMode = newStatus.currentMode.toLowerCase().includes('sensor calibration');
            draftHmi.currentMode = newStatus.currentMode;
        }

        if (!isEqualDeep(prevStatus.maps, newStatus.maps)) {
            draftHmi.maps = newStatus.maps.sort();
        }

        if (!isEqualDeep(prevStatus.currentMap, newStatus.currentMap)) {
            draftHmi.currentMap = newStatus.currentMap;
        }

        if (!isEqualDeep(prevStatus.vehicles, newStatus.vehicles)) {
            draftHmi.vehicles = newStatus.vehicles.sort();
        }

        if (!isEqualDeep(prevStatus.currentVehicle, newStatus.currentVehicle)) {
            draftHmi.currentVehicle = newStatus.currentVehicle;
        }

        if (newStatus.modules) {
            const newKeyList = Object.keys(newStatus.modules || {});
            const curKeyList = Object.keys(prevStatus.modules || {});
            const isValuesChange = () => {
                const newValues = newKeyList.sort().map((key) => newStatus.modules[key]);
                const curValues = curKeyList.sort().map((key) => prevStatus.modules[key]);
                return !isEqualDeep(newValues, curValues);
            };
            if (!isEqualDeep(newKeyList, curKeyList) || isValuesChange()) {
                if (newKeyList.toString() !== curKeyList.toString()) {
                    draftHmi.moduleStatus.clear();
                    draftHmi.moduleStatus = new Map();
                }
                newKeyList.forEach((key) => {
                    draftHmi.moduleStatus.set(key, newStatus.modules[key]);
                });
            }
        }

        if (!isEqualDeep(Object.keys(prevStatus.records || {}), Object.keys(newStatus.records || {}))) {
            draftHmi.records = newStatus.records;
        }

        if (!isEqualDeep(prevStatus.currentRecordStatus, newStatus.currentRecordStatus)) {
            draftHmi.currentRecordStatus = newStatus.currentRecordStatus;
            draftHmi.currentRecordId = newStatus.currentRecordStatus.currentRecordId;
        }

        if (!isEqualDeep(prevStatus.operations, newStatus.operations)) {
            draftHmi.operations = newStatus.operations;
        }

        if (!isEqualDeep(prevStatus.currentOperation, newStatus.currentOperation)) {
            draftHmi.currentOperation = newStatus.currentOperation;
        }

        if (
            !isEqualDeep(
                Object.keys(prevStatus.scenarioSet || {})
                    .sort()
                    .map((key) => prevStatus.scenarioSet?.[key]),
                Object.keys(newStatus.scenarioSet || {})
                    .sort()
                    .map((key) => newStatus.scenarioSet?.[key]),
            )
        ) {
            draftHmi.scenarioSet = newStatus.scenarioSet;
        }

        draftHmi.prevStatus = newStatus;
    },
};
