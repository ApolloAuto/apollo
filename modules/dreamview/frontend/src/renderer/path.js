import STORE from "store";
import PARAMETERS from "store/config/parameters.yml";
import { drawThickBandFromPoints } from "utils/draw";

const PATH_CONFIG = {
    planning_reference_line: {
        width: 0.3,
        color: 0x9999FF,
        opacity: 1,
        zOffset: 5
    },
    DpPolyPathOptimizer: {
        width: 0.6,
        color: 0x59e091,
        opacity: 0.8,
        zOffset: 7
    },
    QpSplinePathOptimizer: {
        width: 0.8,
        color: 0xed6f6f,
        opacity: 0.9,
        zOffset: 6
    },
};

function normalizePlanningPath(path, coordinates) {
    if (!path) {
        return [];
    }

    const result = [];
    for (let i = 0; i < path.length; ++i) {
        const point = path[i];
        const normalizedPoint = coordinates.applyOffset(point);

        if (normalizedPoint === null) {
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

export default class PlanningPath {
    constructor() {
        this.paths = {};
    }

    update(world, coordinates, scene) {
        if (!STORE.options.showPNCMonitor) {
            for (const name in this.paths) {
                this.paths[name].visible = false;
            };
            return;
        }

        // Derive the width of the path ribbon.
        let width = null;
        if (!world.autoDrivingCar.width) {
            console.warn("Unable to get the auto driving car's width, " +
                "planning line width has been set to default: " +
                `${PARAMETERS.planning.defaults.width} m.`);
            width = PARAMETERS.planning.defaults.width;
        } else {
            width = world.autoDrivingCar.width * 0.8;
        }

        if (world.planningData && world.planningData.path) {
            world.planningData.path.forEach((path) => {
                const oldPath = this.paths[path.name];
                if (oldPath) {
                    scene.remove(oldPath);
                    oldPath.geometry.dispose();
                    oldPath.material.dispose();
                }

                const newPath = normalizePlanningPath(path.pathPoint, coordinates);
                this.paths[path.name] = drawThickBandFromPoints(
                    newPath,
                    width * PATH_CONFIG[path.name].width,
                    PATH_CONFIG[path.name].color,
                    PATH_CONFIG[path.name].opacity,
                    PATH_CONFIG[path.name].zOffset);
                scene.add(this.paths[path.name]);
            });
        }
    }
}