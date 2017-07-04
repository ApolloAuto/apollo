import * as THREE from "three";

import STORE from "store";
import PARAMETERS from "store/config/parameters.yml";
import { drawThickBandFromPoints } from "utils/draw";

function normalizePlanningTrajectory(trajectory, coordinates) {
    if (!trajectory) {
        return [];
    }

    const result = [];

    for (let i = 0; i < trajectory.length; ++i) {
        const point = trajectory[i];
        const normalizedPoint = coordinates.applyOffset(
            new THREE.Vector2(point.positionX, point.positionY));

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
        this.path = null;
    }

    update(world, coordinates, scene) {
        if (!STORE.options.showPlanning) {
            if (this.path) {
                this.path.visible = false;
            }
            return;
        }

        // Derive the width of the trajectory ribbon.
        let width = null;
        if (!world.autoDrivingCar.width) {
            console.warn("Unable to get the auto driving car's width, " +
                         "planning line width has been set to default: "
                         `${PARAMETERS.planning.defaults.width} m.`);
            width = PARAMETERS.planning.defaults.width;
        } else {
            width = world.autoDrivingCar.width * 0.8;
        }

        const newPath = normalizePlanningTrajectory(
            world.planningTrajectory, coordinates);

        if (this.path) {
            scene.remove(this.path);
            this.path.geometry.dispose();
            this.path.material.dispose();
        }

        this.path = drawThickBandFromPoints(newPath, width, 0x01D1C1, 0.65, 4);
        scene.add(this.path);
    }
}
