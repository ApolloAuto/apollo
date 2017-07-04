import * as THREE from "three";

import carMaterial from "assets/models/car.mtl";
import carObject from "assets/models/car.obj";
import { loadObject } from "utils/models";

export default class AutoDrivingCar {
    constructor() {
        this.mesh = null;

        // NOTE: loadObject takes some time to update this.mesh. This
        // call is asynchronous.
        loadObject(carMaterial, carObject, {
            x: 1, y: 1, z: 1}, object => {
                this.mesh = object;
                this.mesh.rotation.x = Math.PI / 2;
            });
    }

    update(world, coordinates) {
        const x = world.autoDrivingCar.positionX;
        const y = world.autoDrivingCar.positionY;
        if (!this.mesh || !x || !y) {
            return;
        }

        this.mesh.visible = true;
        const position = coordinates.applyOffset(new THREE.Vector2(x, y));

        if (position === null) {
            return;
        }

        this.mesh.position.set(position.x, position.y, 0);
        this.mesh.rotation.y = world.autoDrivingCar.heading;
    }
}
