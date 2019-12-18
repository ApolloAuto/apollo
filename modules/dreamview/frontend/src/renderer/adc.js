import STORE from "store";

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
    'shadowAdc': {
        menuOptionName: 'showPositionShadow',
        carMaterial: null,
    },
};

const RSS_UNSAFE_MESH = drawImage(iconRssUnsafe, 1.5, 1.5);
const RSS_UNSAFE_MARKER_OFFSET = {
    x: 1,
    y: 1,
    z: 2.6
};

export default class AutoDrivingCar {
    constructor(name, scene) {
        this.mesh = null;
        this.name = name;
        this.rssUnsafeMarker = RSS_UNSAFE_MESH;
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
                this.mesh.visible = false;
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

    updateRssMarker(isRssSafe) {
        this.rssUnsafeMarker.visible = false;
        if (isRssSafe === false && STORE.options.showPlanningRSSInfo) {
            this.rssUnsafeMarker.position.set(this.mesh.position.x + RSS_UNSAFE_MARKER_OFFSET.x,
                                              this.mesh.position.y + RSS_UNSAFE_MARKER_OFFSET.y,
                                              this.mesh.position.z + RSS_UNSAFE_MARKER_OFFSET.z);
            this.rssUnsafeMarker.rotation.set(Math.PI / 2, this.mesh.rotation.y - Math.PI / 2, 0);
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
