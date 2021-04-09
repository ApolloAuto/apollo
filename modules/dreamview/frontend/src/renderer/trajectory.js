import STORE from "store";
import PARAMETERS from "store/config/parameters.yml";
import { drawThickBandFromPoints } from "utils/draw";

const PLANNING_PROPERTIES = {
    planning_reference_line: {
        optionName: 'showPlanningReference',
        width: 0.15,
        color: 0x36A2EB,
        opacity: 1,
        zOffset: 7
    },
    DpPolyPathOptimizer: {
        optionName: 'showPlanningDpOptimizer',
        width: 0.4,
        color: 0x8DFCB4,
        opacity: 0.8,
        zOffset: 6
    },
    QpSplinePathOptimizer: {
        optionName: 'showPlanningQpOptimizer',
        width: 0.65,
        color: 0xd85656,
        opacity: 0.8,
        zOffset: 5
    },
    trajectory: {
        optionName: 'showPlanning',
        width: 0.8,
        color: 0x01D1C1,
        opacity: 0.65,
        zOffset: 4
    }
};

function normalizePlanningTrajectory(trajectory, coordinates) {
    if (!trajectory) {
        return [];
    }

    const result = [];

    for (let i = 0; i < trajectory.length; ++i) {
        const point = trajectory[i];
        const normalizedPoint = coordinates.applyOffset(point);

        if (normalizedPoint === null) {
            // Just skip the trajectory point if it cannot be
            // converted to the local coordinates.
            continue;
        }

        if (result.length > 0) {
            // Skip the point if the interval (against the previous point)
            // is too small. The interval is measured as L1 distance.
            const distance =
                Math.abs(result[result.length - 1].x - normalizedPoint.x) +
                Math.abs(result[result.length - 1].y - normalizedPoint.y);
            if (distance < PARAMETERS.planning.minInterval) {
                continue;
            }
        }

        result.push(normalizedPoint);
    }

    return result;
}

export default class PlanningTrajectory {
    constructor() {
        this.paths = {};
    }

    update(world, planningData, coordinates, scene) {
        // Derive the width of the trajectory ribbon.
        let width = null;
        if (!world.autoDrivingCar.width) {
            console.warn("Unable to get the auto driving car's width, " +
                         "planning line width has been set to default: " +
                         `${PARAMETERS.planning.defaults.width} m.`);
            width = PARAMETERS.planning.defaults.width;
        } else {
            width = world.autoDrivingCar.width;
        }

        // Prepare data
        const newPaths = {};
        if (planningData && planningData.path) {
            planningData.path.forEach((path) => {
                newPaths[path.name] = path.pathPoint;
            });
        }
        if (world.planningTrajectory) {
            newPaths['trajectory'] =
                world.planningTrajectory.map((point) => {
                    return { x: point.positionX, y: point.positionY };
                });
        }

        // Draw paths
        for (const name in PLANNING_PROPERTIES) {
            const property = PLANNING_PROPERTIES[name];
            if (!STORE.options[property.optionName]) {
                if (this.paths[name]) {
                    this.paths[name].visible = false;
                }
            } else {
                const oldPath = this.paths[name];
                if (oldPath) {
                    scene.remove(oldPath);
                    oldPath.geometry.dispose();
                    oldPath.material.dispose();
                }

                if (newPaths[name]) {
                    const points = normalizePlanningTrajectory(newPaths[name], coordinates);
                    this.paths[name] = drawThickBandFromPoints(points,
                        width * property.width, property.color, property.opacity, property.zOffset);
                    scene.add(this.paths[name]);
                }
            }
        }
    }
}