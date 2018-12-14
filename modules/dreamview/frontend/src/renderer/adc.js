import STORE from "store";
import * as THREE from "three";

import carMaterial from "assets/models/car.mtl";
import carObject from "assets/models/car.obj";
import iconRssUnsafe from "assets/images/icons/rss-unsafe.png";
import { loadObject } from "utils/models";
import { drawImage } from "utils/draw";

const CAR_PROPERTIES = {
    'adc': {
        menuOptionName: 'showPositionLocalization',
        carMaterial: carMaterial,
    },
    'planningAdc': {
        menuOptionName: 'showPlanningCar',
        carMaterial: null,
    },
};

const rssUnsafeMesh = drawImage(iconRssUnsafe, 1.5, 1.5, -1.25, 2.7, 0);

export default class AutoDrivingCar {
    constructor(name, scene) {
        this.mesh = null;
        this.name = name;
        this.rssUnsafeMarker = new THREE.Object3D();
        this.rssUnsafeMarker.add(rssUnsafeMesh);
        this.rssUnsafeMarker.mesh = rssUnsafeMesh;
        this.rssUnsafeMarker.visible = false;
        scene.add(this.rssUnsafeMarker);

        const properties = CAR_PROPERTIES[name];
        if (!properties) {
            console.error("Car properties not found for car:", name);
            return;
        }

        // NOTE: loadObject takes some time to update this.mesh.
        // This call is asynchronous.
        loadObject(properties.carMaterial, carObject, {
            x: 1, y: 1, z: 1}, object => {
                this.mesh = object;
                this.mesh.rotation.x = Math.PI / 2;
                this.mesh.visible = STORE.options[properties.menuOptionName];
                scene.add(this.mesh);
            });
    }

    update(coordinates, pose, isRssSafe) {
        if (!this.mesh || !pose || !_.isNumber(pose.positionX) || !_.isNumber(pose.positionY)) {
            return;
        }

        const optionName = CAR_PROPERTIES[this.name].menuOptionName;
        this.mesh.visible = STORE.options[optionName];
        const position = coordinates.applyOffset({x: pose.positionX, y: pose.positionY});
        if (position === null) {
            return;
        }

        this.mesh.position.set(position.x, position.y, 0);
        this.mesh.rotation.y = pose.heading;
        this.rssUnsafeMarker.visible = false;
        if (isRssSafe !== undefined && !isRssSafe && STORE.options.showPlanningRSSInfo) {
            this.rssUnsafeMarker.position.set(position.x, position.y, 0.2);
            this.rssUnsafeMarker.rotation.set(Math.PI / 2, pose.heading - Math.PI / 2, 0);
            this.rssUnsafeMarker.visible = true;
        }
    }

    resizeCarScale(x, y, z) {
        if (!this.mesh) {
            return;
        }
        this.mesh.scale.set(x, y, z);
    }
}
