import * as THREE from 'three';

import { isNumber, isNaN, isArray } from 'lodash';

export default class Coordinates {
    private offset;

    constructor() {
        this.offset = null;
    }

    isInitialized() {
        return this.offset !== null;
    }

    initialize(x, y) {
        if (!isNumber(x) || !isNumber(y)) {
            console.warn('Skip setting invalid offset:', x, y);
            return;
        }

        this.offset = {
            x,
            y,
        };
        console.log(`Offset is set to x:${x}, y:${y}`);
    }

    applyOffset(point, reverse = false) {
        if (this.offset === null) {
            console.error('Offset is not set.');
            return null;
        }
        if (isNaN(this.offset.x) || isNaN(this.offset.y)) {
            console.error('Offset contains NaN!');
            return null;
        }
        if (isNaN(point.x) || isNaN(point.y)) {
            console.warn('Point contains NaN!');
            return null;
        }
        if (!isNaN(point.z)) {
            return new THREE.Vector3(
                reverse ? point.x + this.offset.x : point.x - this.offset.x,
                reverse ? point.y + this.offset.y : point.y - this.offset.y,
                point.z,
            );
        }

        return new THREE.Vector2(
            reverse ? point.x + this.offset.x : point.x - this.offset.x,
            reverse ? point.y + this.offset.y : point.y - this.offset.y,
        );
    }

    applyOffsetToArray(points) {
        if (!isArray(points)) {
            return null;
        }
        return points.map((point) => this.applyOffset(point));
    }
}
