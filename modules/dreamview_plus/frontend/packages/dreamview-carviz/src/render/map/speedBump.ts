import * as THREE from 'three';
import { without } from 'lodash';
import { drawSegmentsFromPoints } from '../../utils/line';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeGroup } from '../../utils/common';

export default class SpeedBump {
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

    drawSpeedBumps(speedBumps) {
        this.currentIds = [];
        if (!speedBumps || speedBumps.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        speedBumps.forEach((speedBump) => {
            const id = speedBump.id.id;
            this.currentIds.push(id);
            if (this.groups[id]) {
                return;
            }
            const lines = speedBump.position;
            const group = new THREE.Group();
            group.name = 'speedBump';
            lines.forEach((line) => {
                line.segment.forEach((segment) => {
                    const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                    const mesh = drawSegmentsFromPoints(points, {
                        color: this.colors.RED,
                        linewidth: 5,
                        zOffset: zOffset.speedBump,
                        opacity: 1,
                    });
                    group.add(mesh);
                });
            });
            this.groups[id] = group;
            this.scene.add(group);
        });
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
