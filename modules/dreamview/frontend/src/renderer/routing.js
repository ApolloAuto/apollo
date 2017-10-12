import * as THREE from "three";

import STORE from "store";

import { drawSegmentsFromPoints } from "utils/draw";

const _ = require('lodash');

export default class Routing {
    constructor() {
        this.routePaths = [];
        this.lastRoutingTime = -1;
    }

    update(world, coordinates, scene) {
        this.routePaths.forEach(path => {
            path.visible = STORE.options.showRouting;
        });
        // There has not been a new routing published since last time.
        if (this.lastRoutingTime === world.routingTime) {
            return;
        }

        this.lastRoutingTime = world.routingTime;

        // Clear the old route paths
        this.routePaths.forEach(path => {
            scene.remove(path);
            path.material.dispose();
            path.geometry.dispose();
        });

        if (world.routePath === undefined) {
            return;
        }

        world.routePath.forEach(path => {
            const points = coordinates.applyOffsetToArray(path.point);
            const pathMesh = drawSegmentsFromPoints(points, 0xFF0000 /* red */,
                10 /* width */ , 5 /* zOffset */, true, true, 0.6 /* opacity */);
            pathMesh.visible = STORE.options.showRouting;
            scene.add(pathMesh);
            this.routePaths.push(pathMesh);
        });
    }
}
