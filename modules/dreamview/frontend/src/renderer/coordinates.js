import * as THREE from "three";

export default class Coordinates {
    constructor() {
        this.offset = null;
    }

    isInitialized() {
        return this.offset !== null;
    }

    initialize(x, y) {
        this.offset = {
            x: x,
            y: y
        };
    }

    applyOffset(point, offset) {
        if (this.offset === null) {
            console.error("Offset is not set.");
            return null;
        } else if (isNaN(point.x) || isNaN(point.y)) {
            // TODO Add warning.
            return null;
        } else if (!isNaN(point.z)) {
            return new THREE.Vector3(
                point.x - this.offset.x,
                point.y - this.offset.y,
                point.z);
        }

        return new THREE.Vector2(point.x - this.offset.x,
                                 point.y - this.offset.y);
    }
}
