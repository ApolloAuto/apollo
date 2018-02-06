import * as THREE from "three";

import { loadTexture } from "utils/models";
import gridGround from "assets/images/ground.png";
import PARAMETERS from "store/config/parameters.yml";

export default class Ground {
    constructor() {
        this.type = "default";
        this.gridOnly = false;
        this.loadedMap = null;
        this.updateMap = null;
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

    initialize(coordinates, isMobileDevice) {
        this.gridOnly = isMobileDevice;
        if (!this.mesh) {
            return false;
        }

        if ((this.loadedMap === this.updateMap || this.gridOnly === true)
            && !this.render(coordinates)) {
            return false;
        }

        this.initialized = true;
        return true;
    }

    update(world, coordinates, scene) {
        if (this.initialized === true && this.gridOnly === false
            && this.loadedMap !== this.updateMap) {
            const dir = this.titleCaseToSnakeCase(this.updateMap);
            const server = 'http://' + window.location.hostname + ':8888';
            const imgUrl = server + '/assets/map_data/' + dir + '/background.jpg';
            loadTexture(imgUrl, texture => {
                console.log("updating ground image with " + dir);
                this.mesh.material.map = texture;
                this.render(coordinates, dir);
            }, err => {
                console.log("using grid as ground image...");
                loadTexture(gridGround, texture => {
                    this.mesh.material.map = texture;
                    this.render(coordinates);
                });
            });
            this.loadedMap = this.updateMap;
        }
    }

    updateImage(mapName) {
        this.updateMap = mapName;
    }

    render(coordinates, mapName = 'defaults') {
        console.log("rendering ground image...");
        const { xres, yres, mpp, xorigin, yorigin } = PARAMETERS.ground[mapName];

        let position = coordinates.applyOffset({x: xorigin, y: yorigin});
        if (position === null) {
            console.warn("Cannot find position for ground mesh!");
            return false;
        }
        // NOTE: Setting the position to (0, 0) makes the center of
        // the ground image to overlap with the offset point, which
        // is the car position on the first received frame.
        if (mapName === 'defaults') {
            position = {x: 0, y: 0};
        }

        this.mesh.position.set(position.x, position.y, 0);
        this.mesh.scale.set(xres * mpp, yres * mpp, 1);
        this.mesh.material.needsUpdate = true;
        this.mesh.overdraw = false;

        return true;
    }

    titleCaseToSnakeCase(str) {
        return str.replace(/\s/g, '_').toLowerCase();
    }
}
