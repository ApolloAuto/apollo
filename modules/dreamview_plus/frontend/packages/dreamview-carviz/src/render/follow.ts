import * as THREE from 'three';
import { planningParams } from '../constant/params';
import { disposeMesh, drawThickBandFromPoints } from '../utils/common';

const DEFAULT_WIDTH = planningParams.defaults.width;
const MIN_INTERVAL = planningParams.minInterval;

function normalizePlanningTrajectory(trajectory) {
    const result = [];
    if (!trajectory || trajectory.length === 0) {
        return [];
    }

    for (let i = 0; i < trajectory.length; i += 1) {
        const point = new THREE.Vector2(trajectory[i].positionX, trajectory[i].positionY);
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

export default class Follow {
    private scene;

    private paths;

    private option;

    private coordinates;

    constructor(scene, coordinates) {
        this.paths = null;
        this.scene = scene;
        this.coordinates = coordinates;
    }

    update(planningData, autoDrivingCar) {
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

        this.dispose();

        let property = planningParams.pathProperties.follow_planning_line;

        if (!property) {
            // console.warn(`No path properties found for [${name}]. Use default properties instead.`);
            property = planningParams.pathProperties.default;
        }

        if (planningData) {
            let points = normalizePlanningTrajectory(planningData);

            points = this.coordinates.applyOffsetToArray(points);

            if (points.length === 0) {
                return;
            }

            const line = drawThickBandFromPoints(points, {
                color: property.color,
                opacity: property.opacity,
                lineWidth: width * property.width,
            });

            if (line) {
                line.position.z = property.zOffset;
                this.paths = line;
            }

            this.scene.add(this.paths);
        }
    }

    dispose() {
        const oldPath = this.paths;
        if (oldPath) {
            disposeMesh(oldPath);
            this.scene.remove(oldPath);
            this.paths = null;
        }
    }
}
