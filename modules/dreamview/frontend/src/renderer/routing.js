import * as THREE from "three";

import STORE from "store";

import { drawSegmentsFromPoints } from "utils/draw";

const _ = require('lodash');

export default class Routing {
    constructor() {
        this.routePath = null;
        this.lastRoutingTime = -1;
    }

    update(world, coordinates, scene) {
        if (this.routePath) {
            this.routePath.visible = STORE.options.showRouting;
        }
        // There has not been a new routing published since last time.
        if (this.lastRoutingTime === world.routingTime) {
            return;
        }

        this.lastRoutingTime = world.routingTime;

        if (this.routePath) {
            scene.remove(this.routePath);
            this.routePath.material.dispose();
            this.routePath.geometry.dispose();
        }

        const points = world.route === undefined ? []
            : coordinates.applyOffsetToArray(world.route);
        this.routePath = drawSegmentsFromPoints(
            points, 0xFF0000 /* red */, 10, 5, true, true, 0.6);
        this.routePath.visible = STORE.options.showRouting;
        scene.add(this.routePath);
    }
}
