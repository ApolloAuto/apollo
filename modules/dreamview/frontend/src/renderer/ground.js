import * as THREE from "three";

import { loadTexture } from "utils/models";
import gridGround from "assets/images/ground.png";
import PARAMETERS from "store/config/parameters.yml";
import STORE from "store";

export default class Ground {
    constructor() {
        this.type = "default";
        this.loadedMap = null;
        this.updateMap = null;
        this.mesh = null;
        this.geometry = null;
        this.initialized = false;
        this.inNaviMode = null;

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

        if (this.loadedMap === this.updateMap && !this.render(coordinates)) {
            return false;
        }

        this.initialized = true;
        return true;
    }

    loadGrid(coordinates) {
        loadTexture(gridGround, texture => {
            console.log("using grid as ground image...");
            this.mesh.material.map = texture;
            this.mesh.type = "grid";
            this.render(coordinates);
        });
    }

    update(world, coordinates, scene) {
        if (this.initialized !== true) {
            return;
        }
        const modeChanged = this.inNaviMode !== STORE.hmi.inNavigationMode;
        this.inNaviMode = STORE.hmi.inNavigationMode;
        if (this.inNaviMode) {
            this.mesh.type = "grid";
            if (modeChanged) {
                this.loadGrid(coordinates);
            }
        } else {
            this.mesh.type = "refelction";
        }

        if (this.mesh.type === "grid") {
            const adc = world.autoDrivingCar;
            const position = coordinates.applyOffset({x: adc.positionX, y: adc.positionY});
            this.mesh.position.set(position.x, position.y, 0);
        } else if (this.loadedMap !== this.updateMap || modeChanged) {
            // Only reload reflection map upon map/mode change.
            const dir = this.titleCaseToSnakeCase(this.updateMap);
            const host = window.location;
            const port = PARAMETERS.server.port;
            const server = `${host.protocol}//${host.hostname}:${port}`;
            const imgUrl = `${server}/assets/map_data/${dir}/background.jpg`;
            loadTexture(imgUrl, texture => {
                console.log("updating ground image with " + dir);
                this.mesh.material.map = texture;
                this.mesh.type = "reflection";
                this.render(coordinates, dir);
            }, err => {
                this.loadGrid(coordinates);
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
