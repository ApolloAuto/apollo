import React, { useRef, useCallback } from 'react';
import { LinearInterpolant } from 'three';
import ChartBase, {
    extractDataPoints,
    generateDataPoints,
} from '@dreamview/dreamview-core/src/components/panels/PncMonitor/Chart/ChartBase';
import { timestampMsToTimeString } from '@dreamview/dreamview-core/src/util/misc';

const MAX_SCENARIO_LENGTH = 5;

const PATH_DISPLAY_NAME: Record<string, string> = {
    planning_reference_line: 'ReferenceLine',
    DpStSpeedOptimizer: 'SpeedHeuristic',
    QpSplineStSpeedOptimizer: 'PlannedSpeed',
};
export const initData: any = () => ({
    vt: {},
    theta: {},
    st: {},
    speedHeuristic: {},
    customChart: [] as any,
    speed: {},
    referenceTheta: {},
    referenceKappa: {},
    referenceDKappa: {},
    kappa: {},
    DKappa: {},
    acceleration: {},
    scenarioHistory: [] as any,
    vehicle: {},
});

export default function usePlanningData() {
    const data = useRef<any>(initData());
    const planningTimeSec = useRef(0);
    const updatePlanningTime = (time: number) => {
        planningTimeSec.current = time;
    };

    function updateVTGraph(stSpeedGraph: any) {
        (stSpeedGraph || []).forEach((stGraph: any) => {
            data.current.vt[stGraph.name] = {};
            const graph = data.current.vt[stGraph.name];
            // speed: limit
            graph.limit = extractDataPoints(stGraph.speedLimit, 's', 'v', false, 0, false);
            // speed: planned
            graph.planned = extractDataPoints(stGraph.speedProfile, 's', 'v', false, 0, false);
            // speed: constraint
            if (stGraph.speedConstraint) {
                const curveT = stGraph.speedProfile.map((point: any) => point.t);
                const curveS = stGraph.speedProfile.map((point: any) => point.s);
                const interpolant = new LinearInterpolant(curveT, curveS, 1, []);
                const speedConstraintS = stGraph.speedConstraint.t.map((point: any) => interpolant.evaluate(point)[0]);
                graph.lowerConstraint = generateDataPoints(speedConstraintS, stGraph.speedConstraint.lowerBound);
                graph.upperConstraint = generateDataPoints(speedConstraintS, stGraph.speedConstraint.upperBound);
            }
        });
    }

    function updateTrajectoryThetaGraph(trajectory: any) {
        data.current.theta.Trajectory = extractDataPoints(trajectory, 'timestampSec', 'heading');
    }

    function updateSTGraph(stGraphs: any) {
        (stGraphs || []).forEach((stGraph: any) => {
            data.current.st[stGraph.name] = { obstaclesBoundary: {} };
            const graph = data.current.st[stGraph.name];

            // obstacle boundary lines
            if (stGraph.boundary) {
                stGraph.boundary.forEach((boundary: any) => {
                    const type = boundary.type.substring('ST_BOUNDARY_TYPE_'.length);
                    const label = `${boundary.name}_${type}`;
                    graph.obstaclesBoundary[label] = extractDataPoints(boundary.point, 't', 's', false, 0, false);
                });
            }

            // curve line
            graph.curveLine = extractDataPoints(stGraph.speedProfile, 't', 's');

            // kernel cruise line
            if (stGraph.kernelCruiseRef) {
                graph.kernelCruise = generateDataPoints(stGraph.kernelCruiseRef.t, stGraph.kernelCruiseRef.cruiseLineS);
            }

            // kernel follow line
            if (stGraph.kernelFollowRef) {
                graph.kernelFollow = generateDataPoints(stGraph.kernelFollowRef.t, stGraph.kernelFollowRef.followLineS);
            }
        });
    }

    function updateSpeedHeuristicGraph(stGraphs: any) {
        (stGraphs || []).forEach((stGraph: any) => {
            if (!stGraph.name) {
                return;
            }
            data.current.speedHeuristic[stGraph.name] = { obstaclesBoundary: {} };
            const graph = data.current.speedHeuristic[stGraph.name];

            // obstacle boundary lines
            if (stGraph.boundary) {
                stGraph.boundary.forEach((boundary: any) => {
                    const type = boundary.type.substring('ST_BOUNDARY_TYPE_'.length);
                    const label = `${boundary.name}_${type}`;
                    graph.obstaclesBoundary[label] = extractDataPoints(boundary.point, 't', 's', false, 0, false);
                });
            }

            // curve line
            graph.curveLine = extractDataPoints(stGraph.speedProfile, 't', 's');

            // kernel cruise line
            if (stGraph.kernelCruiseRef) {
                graph.kernelCruise = generateDataPoints(stGraph.kernelCruiseRef.t, stGraph.kernelCruiseRef.cruiseLineS);
            }

            // kernel follow line
            if (stGraph.kernelFollowRef) {
                graph.kernelFollow = generateDataPoints(stGraph.kernelFollowRef.t, stGraph.kernelFollowRef.followLineS);
            }
        });
    }

    function updateSpeed(speedPlans: any, trajectory: any) {
        (speedPlans || []).forEach((plan: any) => {
            const name = PATH_DISPLAY_NAME[plan.name] || plan.name;
            data.current.speed[name] = extractDataPoints(plan.speedPoint, 't', 'v');
        });

        if (trajectory) {
            data.current.speed.VehicleSpeed = extractDataPoints(
                trajectory,
                'timestampSec',
                'speed',
                false,
                -planningTimeSec.current,
            );
        }
    }

    function updateReferenceTheta(paths: any) {
        paths.forEach((path: any) => {
            const name = PATH_DISPLAY_NAME[path.name] || path.name;
            data.current.referenceTheta[name] = extractDataPoints(path.pathPoint, 's', 'theta');
        });
    }

    function updateReferenceKappaGraph(paths: any) {
        paths.forEach((path: any) => {
            const name = PATH_DISPLAY_NAME[path.name] || path.name;
            data.current.referenceKappa[name] = extractDataPoints(path.pathPoint, 's', 'kappa');
        });
    }

    function updateReferenceDKappa(paths: any) {
        paths
            .filter((path: any) => PATH_DISPLAY_NAME[path.name])
            .forEach((path: any) => {
                const name = PATH_DISPLAY_NAME[path.name];
                data.current.referenceDKappa[name] = extractDataPoints(path.pathPoint, 's', 'dkappa');
            });
    }

    function updateKappaGraph(trajectory: any) {
        data.current.kappa.Trajectory = extractDataPoints(trajectory, 'timestampSec', 'kappa');
    }

    function updateDkappaGraph(trajectory: any) {
        data.current.DKappa.Trajectory = extractDataPoints(trajectory, 'timestampSec', 'dkappa');
    }

    function updateAcceleration(trajectory: any) {
        if (trajectory) {
            data.current.acceleration.VehicleSpeed = extractDataPoints(
                trajectory,
                'timestampSec',
                'speedAcceleration',
                false /* loop back */,
                -planningTimeSec.current,
            );
        }
    }

    function updateScenario(newScenario: any, newTimeInSec: any) {
        if (!newScenario) {
            return;
        }
        const currScenario = data.current.scenarioHistory.slice(-1)[0] || {};

        if (currScenario.timeSec && newTimeInSec < currScenario.timeSec) {
            // new data set, clean up existing one
            data.current.scenarioHistory = [];
        }
        if (
            currScenario.scenarioPluginType !== newScenario.scenarioPluginType ||
            currScenario.stagePluginType !==
                newScenario.stagePluginType.replace(`${newScenario.scenarioPluginType}_`, '')
        ) {
            data.current.scenarioHistory.push({
                timeSec: newTimeInSec,
                timeString: timestampMsToTimeString(newTimeInSec * 1000, true),
                scenarioPluginType: newScenario.scenarioPluginType,
                stagePluginType: newScenario.stagePluginType
                    ? newScenario.stagePluginType.replace(`${newScenario.scenarioPluginType}_`, '')
                    : '-',
            });
            if (data.current.scenarioHistory.length > MAX_SCENARIO_LENGTH) {
                data.current.scenarioHistory.shift();
            }
        }
    }

    function updateCustomChart(chart: any, vehicle: any) {
        data.current.customChart = chart;
        data.current.vehicle = vehicle;
    }

    const onSimData = useCallback((simdata: any) => {
        const planningData = simdata.planningData;
        const newPlanningTime = simdata?.latency?.planning?.timestampSec;
        if (planningData && planningTimeSec.current !== newPlanningTime) {
            if (planningData.chart) {
                updateCustomChart(planningData.chart, simdata.vehicleParam);
            } else {
                data.current.customChart = [];
            }

            if (planningData?.path) {
                updateReferenceTheta(planningData.path);
                updateReferenceKappaGraph(planningData.path);
                updateReferenceDKappa(planningData.path);
            } else {
                data.current.referenceTheta = {};
                data.current.referenceKappa = {};
                data.current.referenceDKappa = {};
            }

            if (planningData?.stGraph) {
                updateSTGraph(planningData.stGraph);
                updateVTGraph(planningData.stGraph);
                updateSpeedHeuristicGraph(planningData.stGraph);
            } else {
                data.current.st = {};
                data.current.vt = {};
                data.current.speedHeuristic = {};
            }

            if (simdata?.planningTrajectory) {
                updateTrajectoryThetaGraph(simdata.planningTrajectory);
                updateKappaGraph(simdata.planningTrajectory);
                updateDkappaGraph(simdata.planningTrajectory);
                updateAcceleration(simdata.planningTrajectory);
            } else {
                data.current.theta.Trajectory = [];
                data.current.kappa.Trajectory = [];
                data.current.DKappa.Trajectory = [];
                data.current.acceleration.VehicleSpeed = [];
            }

            if (planningData?.speedPlan) {
                updateSpeed(planningData?.speedPlan, simdata?.planningTrajectory);
            } else {
                data.current.speed = {};
            }

            if (planningData.scenario) {
                updateScenario(planningData.scenario, newPlanningTime);
            }

            updatePlanningTime(newPlanningTime);
        }
        if (!planningData) {
            data.current = initData();
        }
        return { ...data.current };
    }, []);

    return {
        data,
        onSimData,
    };
}
