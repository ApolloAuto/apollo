import * as THREE from 'three';
import { without } from 'lodash';
import { drawZone } from '../../utils/zone';
import { drawPolygon } from '../../utils/polygon';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeGroup } from '../../utils/common';

export default class PncJunction {
    private groups;

    private scene;

    private currentIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.groups = {};
        this.scene = scene;
        this.coordinates = coordinates;
        this.currentIds = [];
        this.colors = colors?.colorMapping || colorMapping;
    }

    drawPncJunctions(pncJunctions) {
        this.currentIds = [];
        if (!pncJunctions || pncJunctions.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        pncJunctions.forEach((pncJunction) => {
            const id = pncJunction.id.id;
            this.currentIds.push(id);
            if (this.groups[id]) {
                return;
            }
            const pncJunctionGroup = new THREE.Group();
            const points = this.coordinates.applyOffsetToArray(pncJunction.polygon.point);
            const zoneMesh = drawZone(points, {
                color: this.colors.BLUE,
                zOffset: zOffset.pncJunction,
                opacity: 0.3,
                matrixAutoUpdate: true,
            });

            const lineMesh = drawPolygon(points, {
                color: this.colors.BLUE,
                linewidth: 1,
                zOffset: zOffset.pncJunction,
                opacity: 1,
                matrixAutoUpdate: true,
            });

            pncJunctionGroup.add(zoneMesh);
            pncJunctionGroup.add(lineMesh);

            this.groups[id] = pncJunctionGroup;
            this.scene.add(pncJunctionGroup);
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
