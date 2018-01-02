import * as THREE from "three";

import { loadTexture } from "utils/models";
import gridGround from "assets/images/ground.png";
import PARAMETERS from "store/config/parameters.yml";

export default class Ground {
    constructor() {
        this.type = "default";
        this.mesh = null;
        this.geometry = null;
        this.initialized = false;
        loadTexture(gridGround, texture => {
            this.geometry = new THREE.PlaneGeometry(1, 1);
            this.mesh = new THREE.Mesh(
                this.geometry,
                new THREE.MeshBasicMaterial({map: texture}));
        });
    }

    initialize(coordinates) {
        if (!this.mesh) {
            return false;
        }

        const { xres, yres, mpp } = PARAMETERS.ground;

        this.mesh.scale.set(xres * mpp / 1000, yres * mpp / 1000, 1);
        // NOTE: Setting the position to (0, 0) makes the center of
        // the ground image to overlap with the offset point, which
        // in this case is the car position on the first received frame.
        this.mesh.position.set(0, 0, 0);
        this.mesh.overdraw = false;

        this.initialized = true;
        return true;
    }

    update(world, coordinates, scene) {
        // Do nothing in the default ground
    }
}
