/* eslint-disable no-param-reassign */
import cloneDeep from 'lodash/cloneDeep';
import * as TYPES from './actionTypes';

export const hmiUtils = {
    getScenarioName(scenarioSetId: string, scenarioId: string, scenarioSets: any) {
        if (!scenarioSetId || !scenarioId || !scenarioSets) {
            return '';
        }
        const scenarioSet = scenarioSets[scenarioSetId];
        if (!scenarioSet) {
            return '';
        }
        const scenarios = scenarioSet.scenarios || [];
        const scenario = scenarios.find((item: any) => item.scenarioId === scenarioId);
        if (!scenario) {
            return '';
        }
        return scenario.scenarioName;
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
    rotate2DPoint([x, y]: any, rotationInRad: any) {
        const rad = rotationInRad - Math.PI / 2;
        return {
            x: x * Math.cos(rad) - y * Math.sin(rad),
            y: x * Math.sin(rad) + y * Math.cos(rad),
        };
    },
    calculateCarPolygonPoints(positionX: any, positionY: any, headingInRad: any, vehicleParam: any) {
        const config = vehicleParam;
        const polygonPoints: any = [
            [-config.leftEdgeToCenter, config.frontEdgeToCenter],
            [config.rightEdgeToCenter, config.frontEdgeToCenter],
            [config.rightEdgeToCenter, -config.backEdgeToCenter],
            [-config.leftEdgeToCenter, -config.backEdgeToCenter],
        ];
        polygonPoints.forEach((point: any) => {
            const newPoint = hmiUtils.rotate2DPoint(point, headingInRad);
            point[0] = positionX + newPoint.x;
            point[1] = positionY + newPoint.y;
        });
        return polygonPoints;
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

        if (!isEqualDeep(prevStatus.otherComponents, newStatus.otherComponents)) {
            draftHmi.otherComponents = newStatus.otherComponents;
        }

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

        if (!isEqualDeep(prevStatus.currentScenarioSetId, newStatus.currentScenarioSetId)) {
            draftHmi.currentScenarioSetId = newStatus.currentScenarioSetId;
            draftHmi.currentScenarioName = hmiUtils.getScenarioName(
                newStatus.currentScenarioSetId,
                newStatus.currentScenarioId,
                newStatus.scenarioSet,
            );
        }

        if (!isEqualDeep(prevStatus.currentScenarioId, newStatus.currentScenarioId)) {
            draftHmi.currentScenarioId = newStatus.currentScenarioId;
            draftHmi.currentScenarioName = hmiUtils.getScenarioName(
                newStatus.currentScenarioSetId,
                newStatus.currentScenarioId,
                newStatus.scenarioSet,
            );
        }

        if (!isEqualDeep(prevStatus.vehicles, newStatus.vehicles)) {
            draftHmi.vehicles = newStatus.vehicles.sort();
        }

        if (!isEqualDeep(prevStatus.currentVehicle, newStatus.currentVehicle)) {
            draftHmi.currentVehicle = newStatus.currentVehicle;
        }

        if (newStatus.modules) {
            const newKeyList = Object.keys(newStatus.modules ?? {});
            const curKeyList = Object.keys(prevStatus.modules ?? {});
            const isValuesChange = () => newKeyList.some((key) => newStatus.modules[key] !== prevStatus.modules[key]);
            if (!isEqualDeep(newKeyList, curKeyList) || isValuesChange()) {
                newKeyList.forEach((key) => {
                    draftHmi.moduleStatus.set(key, newStatus.modules[key]);
                });
                const newKeySet = new Set(newKeyList);
                curKeyList.forEach((key) => {
                    if (!newKeySet.has(key)) {
                        draftHmi.moduleStatus.delete(key);
                    }
                });
            }
        }

        if (newStatus.modulesLock) {
            const newKeyList = Object.keys(newStatus.modulesLock ?? {});
            const curKeyList = Object.keys(prevStatus.modulesLock ?? {});
            const isValuesChange = () =>
                newKeyList.some((key) => newStatus.modulesLock[key] !== prevStatus.modulesLock[key]);
            if (!isEqualDeep(newKeyList, curKeyList) || isValuesChange()) {
                newKeyList.forEach((key) => {
                    draftHmi.modulesLock.set(key, newStatus.modulesLock[key]);
                });
                const newKeySet = new Set(newKeyList);
                curKeyList.forEach((key) => {
                    if (!newKeySet.has(key)) {
                        draftHmi.modulesLock.delete(key);
                    }
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

        if (!isEqualDeep(prevStatus.dataRecorderComponent, newStatus.dataRecorderComponent)) {
            draftHmi.dataRecorderComponent = newStatus.dataRecorderComponent;
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
            draftHmi.currentScenarioName = hmiUtils.getScenarioName(
                newStatus.currentScenarioSetId,
                newStatus.currentScenarioId,
                newStatus.scenarioSet,
            );
        }

        draftHmi.backendShutdown = newStatus.backendShutdown;

        draftHmi.prevStatus = newStatus;
    },
};
