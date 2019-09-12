import _ from 'lodash';
import STORE from "store";
import { drawThickBandFromPoints, drawDashedLineFromPoints } from "utils/draw";

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
        if (world.planningTrajectory) {
            newPaths['trajectory'] =
                world.planningTrajectory.map((point) => {
                    return { x: point.positionX, y: point.positionY };
                });
        }
        if (planningData && planningData.path) {
            planningData.path.forEach((path) => {
                newPaths[path.name] = path.pathPoint;
            });
        }

        // Draw paths
        const allPaths = _.union(Object.keys(this.paths), Object.keys(newPaths));
        allPaths.forEach((name) => {
            const optionName = name === 'trajectory' ? 'showPlanningTrajectory' : name;
            if (!STORE.options[optionName] && !STORE.options.customizedToggles.get(optionName)) {
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

                let property = PARAMETERS.planning.pathProperties[name];
                if (!property) {
                    console.warn(
                        `No path properties found for [${name}]. Use default properties instead.`);
                    property = PARAMETERS.planning.pathProperties.default;
                    PARAMETERS.planning.pathProperties[name] = property;
                }

                if (newPaths[name]) {
                    const points = normalizePlanningTrajectory(newPaths[name], coordinates);
                    if (property.style === 'dash') {
                        this.paths[name] = drawDashedLineFromPoints(points, property.color,
                            width * property.width, 1 /* dash size */, 1 /* gapSize */,
                            property.zOffset, property.opacity);
                    } else {
                        this.paths[name] = drawThickBandFromPoints(points, width * property.width,
                            property.color, property.opacity, property.zOffset);
                    }
                    scene.add(this.paths[name]);
                }
            }
        });
    }
}