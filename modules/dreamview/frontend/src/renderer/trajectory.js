import _ from 'lodash';
import STORE from "store";
import * as THREE from "three";
import { drawThickBandFromPoints, drawSegmentsFromPoints, drawCircle, drawArrow } from "utils/draw";

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

function getPullOverStatus({ lengthFront, lengthBack, widthLeft, widthRight }) {
    const pullOverStatus = new THREE.Group();
    const color = 0x006AFF;
    const polygon = drawSegmentsFromPoints(
        [
            new THREE.Vector3(lengthFront, -widthLeft, 0),
            new THREE.Vector3(lengthFront, widthRight, 0),
            new THREE.Vector3(-lengthBack, widthRight, 0),
            new THREE.Vector3(-lengthBack, -widthLeft, 0),
            new THREE.Vector3(lengthFront, -widthLeft, 0),
        ],
        color,
        2,
        5,
    );
    pullOverStatus.add(polygon);

    const material = new THREE.MeshBasicMaterial({
        color,
        transparent: false,
        opacity: 0.5
    });
    const circle = drawCircle(0.2, material);
    pullOverStatus.add(circle);

    const heading = drawArrow(1.5, 2, 0.5, 0.5, color);
    heading.rotation.set(0, 0, -Math.PI / 2);
    pullOverStatus.add(heading);

    return pullOverStatus;
}

export default class PlanningTrajectory {
    constructor() {
        this.paths = {};
        this.pullOverStatus = null;
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
        let propertyIndex = 0;
        const totalPaths = _.union(Object.keys(this.paths), Object.keys(newPaths));
        totalPaths.forEach((name) => {
            const optionName = name === 'trajectory' ? 'showPlanningTrajectory': name;
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
                if (propertyIndex >= PARAMETERS.planning.pathProperties.length) {
                    console.error("No enough property to render the planning path, " +
                                  "use a duplicated property instead.");
                    propertyIndex = 0;
                }
                const property = PARAMETERS.planning.pathProperties[propertyIndex];
                if (newPaths[name]) {
                    const points = normalizePlanningTrajectory(newPaths[name], coordinates);
                    this.paths[name] = drawThickBandFromPoints(points,
                        width * property.width, property.color, property.opacity, property.zOffset);
                    scene.add(this.paths[name]);
                }
            }
            propertyIndex += 1;
        });

        // Draw pull over status
        if (planningData) {
            const { pullOverStatus } = planningData;
            if (!STORE.options.customizedToggles.get('pullOverStatus')) {
                if (this.pullOverStatus) {
                    this.pullOverStatus.traverse((child) => {
                        child.visible = false;
                    });
                }
            } else {
                if (this.pullOverStatus) {
                    this.pullOverStatus.traverse((child) => {
                        if (child.geometry !== undefined) {
                            child.geometry.dispose();
                            child.material.dispose();
                        }
                        scene.remove(this.pullOverStatus);
                    });
                }
                if (pullOverStatus) {
                    this.pullOverStatus = getPullOverStatus(pullOverStatus);

                    const position = coordinates.applyOffset({
                        x: pullOverStatus.position.x,
                        y: pullOverStatus.position.y,
                        z: 0.3,
                    });
                    this.pullOverStatus.position.set(position.x, position.y, position.z);
                    this.pullOverStatus.rotation.set(0, 0, pullOverStatus.theta);
                    scene.add(this.pullOverStatus);
                }
            }
        }
    }
}