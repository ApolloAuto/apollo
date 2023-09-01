import * as THREE from 'three';
import { union } from 'lodash';
import { planningParams } from '../constant/params';
import { disposeMesh, drawThickBandFromPoints } from '../utils/common';
import { drawDashedLineFromPoints } from '../utils/line';
import { zOffset } from '../constant/common';

const DEFAULT_WIDTH = planningParams.defaults.width;
const MIN_INTERVAL = planningParams.minInterval;

function normalizePlanningTrajectory(trajectory) {
    const result = [];
    if (!trajectory || trajectory.length === 0) {
        return [];
    }

    for (let i = 0; i < trajectory.length; i += 1) {
        const point = new THREE.Vector2(trajectory[i].x, trajectory[i].y);
        if (!point) {
            continue;
        }

        if (result.length > 0) {
            const lastPoint = result[result.length - 1];
            const distance = Math.abs(lastPoint.x - point.x) + Math.abs(lastPoint.y - point.y);
            if (distance < MIN_INTERVAL) {
                continue;
            }
        }
        result.push(point);
    }
    if (result.length < 2) {
        return [];
    }

    return result;
}

export default class Planning {
    private scene;

    private paths;

    private option;

    private coordinates;

    constructor(scene, option, coordinates) {
        this.scene = scene;
        this.option = option;
        this.coordinates = coordinates;
        this.paths = {};
    }

    update(planningTrajectory, planningData, autoDrivingCar) {
        if (!this.coordinates.isInitialized()) {
            return;
        }
        let width = null;
        if (!autoDrivingCar?.width) {
            console.warn(
                `Unable to get the auto driving car's width, planning line width has been set to default: ${DEFAULT_WIDTH} m.`,
            );
            width = DEFAULT_WIDTH;
        } else {
            width = autoDrivingCar.width;
        }

        const newPaths = {
            trajectory: [],
        };
        if (planningTrajectory) {
            newPaths.trajectory = planningTrajectory.map((point) => ({ x: point.positionX, y: point.positionY }));
        }
        if (planningData && planningData.path) {
            planningData.path.forEach((path) => {
                newPaths[path.name] = path.pathPoint;
            });
        }

        const allPaths = union(Object.keys(this.paths), Object.keys(newPaths));
        allPaths.forEach((name) => {
            const oldPath = this.paths[name];
            if (name === 'trajectory' && !this.option.layerOption.Planning.planningTrajectory) {
                disposeMesh(oldPath);
                this.scene.remove(oldPath);
                delete this.paths[name];
                return;
            }
            if (oldPath) {
                disposeMesh(oldPath);
                this.scene.remove(oldPath);
                delete this.paths[name];
            }

            let property = planningParams.pathProperties[name];
            if (!property) {
                console.warn(`No path properties found for [${name}]. Use default properties instead.`);
                property = planningParams.pathProperties.default;
            }

            if (newPaths[name]) {
                let points = normalizePlanningTrajectory(newPaths[name]);
                points = this.coordinates.applyOffsetToArray(points);
                if (points.length === 0) {
                    return;
                }
                if (property.style === 'dash') {
                    this.paths[name] = drawDashedLineFromPoints(points, {
                        color: property.color,
                        linewidth: width * property.width,
                        dashSize: 1,
                        gapSize: 1,
                        zOffset: property.zOffset,
                        opacity: property.opacity,
                        matrixAutoUpdate: true,
                    });
                } else {
                    const line = drawThickBandFromPoints(points, {
                        color: property.color,
                        opacity: property.opacity,
                        lineWidth: width * property.width,
                    });
                    if (line) {
                        line.position.z = property.zOffset;
                        this.paths[name] = line;
                    }
                }
                this.scene.add(this.paths[name]);
            }
        });
    }

    dispose() {
        Object.keys(this.paths).forEach((key) => {
            const mesh = this.paths[key];
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.paths = {};
    }
}
