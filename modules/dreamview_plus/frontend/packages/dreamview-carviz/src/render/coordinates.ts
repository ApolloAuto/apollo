import * as THREE from 'three';
import { isNumber, isNaN, isArray } from 'lodash';
import { Nullable } from '@dreamview/dreamview-carviz/src/utils/similarFunctions';

interface IOffset {
    x: number;
    y: number;
}

export default class Coordinates {
    private offset: Nullable<IOffset>;

    constructor() {
        this.offset = null;
    }

    get currentOffset(): Nullable<IOffset> {
        if (!this.isInitialized()) {
            console.error('Offset is not set.');
            return null;
        }
        return this.offset;
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

    /**
     * Apply offset to a point.
     * @param point Point to apply offset.
     * @param reverse If true, apply reverse offset.
     */
    applyOffset(point, reverse = false) {
        if (this.offset === null) {
            // console.error('Offset is not set.');
            return null;
        }
        if (isNaN(this.offset?.x) || isNaN(this.offset?.y)) {
            console.error('Offset contains NaN!');
            return null;
        }
        if (isNaN(point?.x) || isNaN(point?.y)) {
            console.warn('Point contains NaN!');
            return null;
        }
        if (!isNaN(point?.z)) {
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

    offsetToVector3(offset: IOffset): THREE.Vector3 {
        return new THREE.Vector3(offset.x, offset.y, 0);
    }
}
