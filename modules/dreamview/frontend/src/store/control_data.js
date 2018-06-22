import { action, observable } from 'mobx';
import { LinearInterpolant, EllipseCurve } from 'three';

const MAX_HISTORY_POINTS = 80;

export default class ControlData {
    @observable lastUpdatedTime = null;

    data = this.initData();

    @action updateTime(newTime) {
        this.lastUpdatedTime = newTime;
    }

    initData() {
        return {
            trajectoryGraph: {
                plan: [],
                target: [],
                real: [],
                autoModeZone: [],
                steerCurve: [],
                pose: [{ x: null, y: null, rotation: null }],
            },
            speedGraph: {
                plan: [],
                target: [],
                real: [],
                autoModeZone: []
            },
            curvatureGraph: {
                plan: [],
                target: [],
                real: [],
                autoModeZone: []
            },
            accelerationGraph: {
                plan: [],
                target: [],
                real: [],
                autoModeZone: []
            },
            stationErrorGraph: {
                error: [],
            },
            headingErrorGraph: {
                error: [],
            },
            lateralErrorGraph: {
                error: [],
            },
        };
    }

    updateErrorGraph(graph, currentTimestamp, error) {
        if (!error || !currentTimestamp || !graph) {
            return;
        }

        // clean up data if needed
        const removeAllPoints = graph.error.length > 0 &&
            currentTimestamp < graph.error[graph.error.length - 1].x;
        const removeOldestPoint = (graph.error.length >= MAX_HISTORY_POINTS);
        if (removeAllPoints) {
            graph.error = [];
        } else if (removeOldestPoint) {
            graph.error.shift();
        }

        // add new data
        const hasNewData = graph.error.length === 0 ||
            currentTimestamp !== graph.error[graph.error.length - 1].x;
        if (hasNewData) {
            graph.error.push({ x: currentTimestamp, y: error });
        }
    }

    updateSteerCurve(graph, adc, vehicleParam) {
        const steeringAngle = adc.steeringAngle / vehicleParam.steerRatio;
        let R = null;
        if (Math.abs(Math.tan(steeringAngle)) > 0.0001) {
            R = vehicleParam.length / Math.tan(steeringAngle);
        } else {
            R = 100000;
        }

        const heading = adc.heading;
        const radius = Math.abs(R);
        const lengthAngle = 7200 / (2 * Math.PI * radius) * Math.PI / 180;
        let theta1 = null;
        let theta2 = null;
        let centerangle = null;
        let startangle = null;
        if (R >= 0) {
            centerangle = Math.PI / 2 + heading;
            startangle = (heading - Math.PI / 2);
            theta1 = 0;
            theta2 = lengthAngle;
        } else {
            centerangle = heading - Math.PI / 2;
            startangle = (Math.PI / 2 + heading);
            theta1 = -lengthAngle;
            theta2 = 0;
        }

        const centerx = adc.positionX + Math.cos(centerangle) * radius;
        const centery = adc.positionY + Math.sin(centerangle) * radius;

        const aClockwise = false;
        const curve = new EllipseCurve( // all in radians
            centerx, centery,
            radius, radius,
            theta1, theta2,
            aClockwise,
            startangle
        );

        graph.steerCurve = curve.getPoints(25);
    }

    interpolateValueByCurrentTime(trajectory, currentTime, fieldName) {
        if (fieldName === 'timestampSec') {
            return currentTime;
        } else {
            const absoluteTimes = trajectory.map((point) => {
                return point.timestampSec;
            });
            const plannedValues = trajectory.map((point) => {
                return point[fieldName];
            });
            const interpolant =
                new LinearInterpolant(absoluteTimes, plannedValues, 1, []);
            return interpolant.evaluate(currentTime)[0];
        }
    }

    updateAdcStatusGraph(graph, trajectory, adc, xFieldName, yFieldName) {
        const currentTimestamp = adc.timestampSec;

        // clean up data if needed
        const removeAllPoints = graph.target.length > 0 &&
            currentTimestamp < graph.target[graph.target.length - 1].t;
        const removeOldestPoint = (graph.target.length >= MAX_HISTORY_POINTS);
        if (removeAllPoints) {
            graph.target = [];
            graph.real = [];
            graph.autoModeZone = [];
        } else if (removeOldestPoint) {
            graph.target.shift();
            graph.real.shift();
            graph.autoModeZone.shift();
        }

        const hasNewData = graph.target.length === 0 ||
            currentTimestamp !== graph.target[graph.target.length - 1].t;
        if (hasNewData) {
            // set planned data
            graph.plan = trajectory.map((point) => {
                return { x: point[xFieldName], y: point[yFieldName] };
            });

            // add target value
            graph.target.push({
                x: this.interpolateValueByCurrentTime(
                    trajectory, currentTimestamp, xFieldName),
                y: this.interpolateValueByCurrentTime(
                    trajectory, currentTimestamp, yFieldName),
                t: currentTimestamp,
            });

            // add real value
            graph.real.push({ x: adc[xFieldName], y: adc[yFieldName] });

            // add auto-mode indicator
            const isCompleteAuto = (adc.disengageType === "DISENGAGE_NONE");
            graph.autoModeZone.push({
                x: adc[xFieldName],
                y: isCompleteAuto ? adc[yFieldName] : undefined,
            });
        }
    }

    update(world, vehicleParam) {
        const trajectory = world.planningTrajectory;
        const adc = world.autoDrivingCar;
        if (trajectory && adc) {
            this.updateAdcStatusGraph(this.data.speedGraph,
                trajectory, adc, 'timestampSec', 'speed');
            this.updateAdcStatusGraph(this.data.accelerationGraph,
                trajectory, adc, 'timestampSec', 'speedAcceleration');
            this.updateAdcStatusGraph(this.data.curvatureGraph,
                trajectory, adc, 'timestampSec', 'kappa');

            this.updateAdcStatusGraph(this.data.trajectoryGraph,
                trajectory, adc, 'positionX', 'positionY');
            this.updateSteerCurve(this.data.trajectoryGraph, adc, vehicleParam);
            this.data.trajectoryGraph.pose[0].x = adc.positionX;
            this.data.trajectoryGraph.pose[0].y = adc.positionY;
            this.data.trajectoryGraph.pose[0].rotation = adc.heading;

            this.updateTime(world.planningTime);
        }

        if (world.controlData) {
            const data = world.controlData;
            const timestamp = data.timestampSec;
            this.updateErrorGraph(this.data.stationErrorGraph, timestamp, data.stationError);
            this.updateErrorGraph(this.data.lateralErrorGraph, timestamp, data.lateralError);
            this.updateErrorGraph(this.data.headingErrorGraph, timestamp, data.headingError);
        }
    }
}