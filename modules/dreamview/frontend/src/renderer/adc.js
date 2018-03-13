import STORE from "store";

import carMaterial from "assets/models/car.mtl";
import carObject from "assets/models/car.obj";
import { loadObject } from "utils/models";

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

export default class AutoDrivingCar {
    constructor(name, scene) {
        this.mesh = null;
        this.name = name;

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

    update(coordinates, pose) {
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
    }
}
