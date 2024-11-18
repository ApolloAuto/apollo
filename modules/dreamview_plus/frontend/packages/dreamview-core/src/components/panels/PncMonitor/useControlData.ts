import React, { useRef, useCallback } from 'react';
import { EllipseCurve } from 'three';
import ChartBase, {
    interpolateValueByCurrentTime,
} from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';
import { hmiUtils } from '@dreamview/dreamview-core/src/store/HmiStore';
import lodashLast from 'lodash/last';

const MAX_HISTORY_POINTS = 80;

export const initData = (): any => ({
    trajectory: {
        plan: [],
        target: [],
        real: [],
        autoModeZone: [],
        steerCurve: [],
        currentTargetPoint: [],
    },
    stationError: [],
    lateralError: [],
    headingError: [],
    speed: {
        plan: [],
        target: [],
        real: [],
        autoModeZone: [],
    },
    curvature: {
        plan: [],
        target: [],
        real: [],
        autoModeZone: [],
    },
    acceleration: {
        plan: [],
        target: [],
        real: [],
        autoModeZone: [],
    },
    polygon: {},
    pose: {},
    currentTargetPoint: [],
});
function pushIfChange(arr: any, item: any) {
    const lastTarget: any = lodashLast(arr) || [];
    if (item[0] !== lastTarget[0] && item[1] !== lastTarget[1]) {
        arr.push(item);
    }
}
const init = initData();
export default function useControlData() {
    const data = useRef<any>(init);

    const updatePose = useCallback((adc: any) => {
        data.current.pose.x = adc.positionX;
        data.current.pose.y = adc.positionY;
        data.current.pose.heading = adc.heading;
    }, []);

    const adcStatusMemo = useRef<any>({});
    function updateAdcStatusGraph(graph: any, trajectory: any, adc: any, xFieldName: any, yFieldName: any) {
        const currentTimestamp = adc.timestampSec;
        if (!adcStatusMemo.current[yFieldName]) {
            adcStatusMemo.current[yFieldName] = {};
        }
        const hasNewData = !adcStatusMemo.current[yFieldName][currentTimestamp];
        if (hasNewData) {
            adcStatusMemo.current[yFieldName][currentTimestamp] = true;
            // add target value
            graph.target.push([
                currentTimestamp,
                interpolateValueByCurrentTime(trajectory, currentTimestamp, yFieldName),
            ]);
            // add real value
            graph.real.push([adc[xFieldName], adc[yFieldName]]);
            // add auto-mode indicator
            const isCompleteAuto = adc.disengageType === 'DISENGAGE_NONE';
            graph.autoModeZone.push([currentTimestamp, isCompleteAuto ? adc[yFieldName] : '']);
        }

        const trajectoryMemo: Record<any, boolean> = {};

        graph.plan = (trajectory || [])
            .map((point: any) => [point.timestampSec, point[yFieldName]])
            .filter((item: any) => {
                const key = item[0];
                if (trajectoryMemo[key]) {
                    return false;
                }
                trajectoryMemo[key] = true;
                return key > currentTimestamp;
            })
            .sort(([timeNext]: [number], [timePrev]: [number]) => timeNext - timePrev);
        const planningStartTime = graph.plan?.[1]?.[0];
        graph.target = graph.target
            .sort(([timeNext]: any, [timePrev]: any) => timeNext - timePrev)
            .filter(([x]: any) => x < planningStartTime);
        graph.real = graph.real
            .sort(([timeNext]: any, [timePrev]: any) => timeNext - timePrev)
            .filter(([x]: any) => x < planningStartTime);
        graph.autoModeZone = graph.autoModeZone
            .sort(([timeNext]: any, [timePrev]: any) => timeNext - timePrev)
            .filter(([x]: any) => x < planningStartTime);
    }

    function updateTrajectoryGraph(graph: any, trajectory: any, adc: any, xFieldName: any, yFieldName: any) {
        const currentTimestamp = adc.timestampSec;

        // clean up data if needed
        const removeAllPoints = graph.target.length > 0 && currentTimestamp < graph.target[graph.target.length - 1].t;
        const removeOldestPoint = graph.target.length >= MAX_HISTORY_POINTS;
        if (removeAllPoints) {
            graph.target = [];
            graph.real = [];
            graph.autoModeZone = [];
        } else if (removeOldestPoint) {
            graph.target.shift();
            graph.real.shift();
            graph.autoModeZone.shift();
        }

        const hasNewData = graph.target.length === 0 || currentTimestamp !== graph.target[graph.target.length - 1].t;
        if (hasNewData) {
            // set planned data

            graph.plan = (trajectory || []).reduce((result: any, point: any) => {
                pushIfChange(result, [point[xFieldName], point[yFieldName]]);
                return result;
            }, []);

            // add target value
            pushIfChange(graph.target, [
                interpolateValueByCurrentTime(trajectory, currentTimestamp, xFieldName),
                interpolateValueByCurrentTime(trajectory, currentTimestamp, yFieldName),
            ]);

            // add real value
            pushIfChange(graph.real, [adc[xFieldName], adc[yFieldName]]);

            // add auto-mode indicator
            const isCompleteAuto = adc.disengageType === 'DISENGAGE_NONE';
            pushIfChange(graph.autoModeZone, [adc[xFieldName], isCompleteAuto ? adc[yFieldName] : undefined]);
        }
    }
    function updateCar(vehicleParam: any) {
        // draw ego-vehicle bounding box
        data.current.polygon = hmiUtils.calculateCarPolygonPoints(
            data.current.pose.x,
            data.current.pose.y,
            data.current.pose.heading,
            vehicleParam,
        );
    }

    function updateSteerCurve(graph: any, adc: any, vehicleParam: any) {
        const steeringAngle = adc.steeringAngle / vehicleParam.steerRatio;
        let R = null;
        if (Math.abs(Math.tan(steeringAngle)) > 0.0001) {
            R = vehicleParam.wheelBase / Math.tan(steeringAngle);
        } else {
            R = 100000;
        }

        const heading = adc.heading;
        const radius = Math.abs(R);
        const lengthAngle = ((7200 / (2 * Math.PI * radius)) * Math.PI) / 180;
        let theta1 = null;
        let theta2 = null;
        let centerangle = null;
        let startangle = null;
        if (R >= 0) {
            centerangle = Math.PI / 2 + heading;
            startangle = heading - Math.PI / 2;
            theta1 = 0;
            theta2 = lengthAngle;
        } else {
            centerangle = heading - Math.PI / 2;
            startangle = Math.PI / 2 + heading;
            theta1 = -lengthAngle;
            theta2 = 0;
        }

        const centerx = adc.positionX + Math.cos(centerangle) * radius;
        const centery = adc.positionY + Math.sin(centerangle) * radius;

        const aClockwise = false;
        const curve = new EllipseCurve(centerx, centery, radius, radius, theta1, theta2, aClockwise, startangle);

        graph.steerCurve = curve.getPoints(25).map((item) => [item.x, item.y]) as any;
    }

    function setCurrentTargetPoint(trajectoryPoint: any) {
        data.current.trajectory.currentTargetPoint = [];
        if (trajectoryPoint && trajectoryPoint.pathPoint) {
            pushIfChange(data.current.trajectory.currentTargetPoint, [
                trajectoryPoint.pathPoint.x,
                trajectoryPoint.pathPoint.y,
            ]);
        }
    }

    const memo = useRef<any>({});
    function updateError(graphName: string, currentTimestamp: any, error: any) {
        if (!error || !currentTimestamp) {
            return;
        }
        if (!memo.current[graphName]) {
            memo.current[graphName] = {};
        }
        const dataKey = `${graphName}_data`;
        if (!memo.current[dataKey]) {
            memo.current[dataKey] = [];
        }
        // add new data
        const hasNewData = !memo.current[graphName][currentTimestamp];
        if (hasNewData) {
            memo.current[graphName][currentTimestamp] = true;
            memo.current[dataKey].push([currentTimestamp, error]);
        }
        data.current[graphName] = memo.current[dataKey]
            .sort(([timeNext]: any, [timePrev]: any) => timeNext - timePrev)
            .filter(([x]: any) => x < currentTimestamp);
    }

    const onSimData = useCallback((simdata: any) => {
        const trajectory = simdata.planningTrajectory;
        const adc = simdata.autoDrivingCar;

        setCurrentTargetPoint(simdata.controlData?.currentTargetPoint);
        if (adc) {
            updatePose(adc);
            updateSteerCurve(data.current.trajectory, adc, simdata.vehicleParam);
        }

        if (simdata.vehicleParam) {
            updateCar(simdata.vehicleParam);
        }

        if (trajectory && adc) {
            updateTrajectoryGraph(data.current.trajectory, trajectory, adc, 'positionX', 'positionY');
            updateAdcStatusGraph(data.current.speed, trajectory, adc, 'timestampSec', 'speed');
            updateAdcStatusGraph(data.current.curvature, trajectory, adc, 'timestampSec', 'kappa');
            updateAdcStatusGraph(data.current.acceleration, trajectory, adc, 'timestampSec', 'speedAcceleration');
        }

        if (simdata.controlData) {
            const control = simdata.controlData;
            const timestamp = control.timestampSec;
            updateError('stationError', timestamp, control.stationError);
            updateError('lateralError', timestamp, control.lateralError);
            updateError('headingError', timestamp, control.headingError);
        }

        return { ...data.current };
    }, []);

    const refresh = useCallback((chartNames: string[]) => {
        const newData = initData();
        chartNames.forEach((chartName) => {
            data.current[chartName] = newData[chartName];
        });
    }, []);

    return {
        data,
        onSimData,
        onRefresh: refresh,
    };
}
