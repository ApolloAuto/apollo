/* eslint-disable no-param-reassign */
import cloneDeep from 'lodash/cloneDeep';
import isEqual from 'lodash/isEqual';
import isArray from 'lodash/isArray';
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
    if (Array.isArray(prev) && Array.isArray(next)) {
        return prev.sort().toString() === next.sort().toString();
    }
    return isEqual(prev, next);
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
    changeRTKRecorder(rtkRecorderId: string) {
        console.log('recorderId', rtkRecorderId);
    },
    changeOperate(draftHmi: any, operate: string) {
        draftHmi.currentOperation = operate;
    },
    updateCurrentMode: (draftHmi: any, mode: string) => {
        draftHmi.currentMode = mode;
    },
    toggleModule(hmi: TYPES.IInitState, prop: { key: string }) {
        console.log('module', prop);
    },
    updateStatusSimp: (originHmi: any, draftHmi: any, newStatus: any) => {
        const prevStatus = cloneDeep(originHmi.prevStatus);
        Object.keys(newStatus).forEach((key: string) => {
            const prevValue = prevStatus[key];
            const newValue = newStatus[key];
            if (isEqual(prevValue, newValue)) {
                return;
            }
            if (['currentScenarioSetId', 'currentScenarioId', 'scenarioSet'].includes(key)) {
                draftHmi[key] = newStatus[key];
                draftHmi.currentScenarioName = hmiUtils.getScenarioName(
                    newStatus.currentScenarioSetId,
                    newStatus.currentScenarioId,
                    newStatus.scenarioSet,
                );
            } else if (['modules', 'modulesLock'].includes(key)) {
                draftHmi[key] = new Map(Object.entries(newValue).sort());
            } else if (key === 'currentRecordStatus') {
                draftHmi.currentRecordId = newStatus.currentRecordStatus?.currentRecordId;
                draftHmi.currentRecordStatus = newStatus.currentRecordStatus;
            } else if (isArray(prevValue) && isArray(newValue)) {
                draftHmi[key] = newStatus[key].sort();
            } else {
                draftHmi[key] = newStatus[key];
            }
        });

        draftHmi.backendShutdown = newStatus.backendShutdown;

        draftHmi.prevStatus = newStatus;
    },
};
