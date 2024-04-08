import * as THREE from 'three';
import { without } from 'lodash';
import { drawZone } from '../../utils/zone';
import { drawPolygon } from '../../utils/polygon';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeGroup } from '../../utils/common';

export default class ClearArea {
    private groups;

    private scene;

    private currentIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.scene = scene;
        this.coordinates = coordinates;
        this.groups = {};
        this.currentIds = [];
        this.colors = colors?.colorMapping || colorMapping;
    }

    drawCrosswalk(crosswalks) {
        this.currentIds = [];
        if (!crosswalks || crosswalks.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        crosswalks.forEach((crosswalk) => {
            const id = crosswalk.id.id;
            this.currentIds.push(id);
            if (this.groups[id]) {
                return;
            }
            const group = new THREE.Group();
            const points = this.coordinates.applyOffsetToArray(crosswalk.polygon.point);
            const zoneMesh = drawZone(points, {
                color: this.colors.PURE_WHITE,
                zOffset: zOffset.clearArea,
                opacity: 0.3,
                matrixAutoUpdate: true,
            });
            const lineMesh = drawPolygon(points, {
                color: this.colors.YELLOW,
                linewidth: 1,
                zOffset: zOffset.clearArea,
                opacity: 1,
                matrixAutoUpdate: true,
            });
            group.add(zoneMesh);
            group.add(lineMesh);
            this.groups[id] = group;
            this.scene.add(group);
        });
        this.removeOldGroups();
    }

    dispose() {
        Object.values(this.groups).forEach((group) => {
            disposeGroup(group);
            this.scene.remove(group);
        });
        this.groups = [];
        this.currentIds = [];
    }

    removeOldGroups() {
        const needRemovedIds = without(Object.keys(this.groups), ...this.currentIds);
        if (needRemovedIds && needRemovedIds.length) {
            needRemovedIds.forEach((id) => {
                const removedGroup = this.groups[id];
                disposeGroup(removedGroup);
                this.scene.remove(removedGroup);
                delete this.groups[id];
            });
        }
    }
}
