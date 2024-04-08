import * as THREE from 'three';
import { without } from 'lodash';
import { drawLaneMesh } from '../../utils/line';
import { disposeGroup } from '../../utils/common';
import { colorMapping } from '../../constant/common';

export default class Road {
    private roadMap;

    private scene;

    private currentRoadIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.scene = scene;
        this.coordinates = coordinates;
        this.roadMap = {};
        this.currentRoadIds = [];
        this.colors = colors;
    }

    drawRoads(roads) {
        this.currentRoadIds = [];
        if (!roads || roads.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        roads.forEach((road) => {
            const id = road.id.id;
            this.currentRoadIds.push(id);
            if (this.roadMap[id]) {
                return;
            }
            const group = new THREE.Group();
            road.section.forEach((section) => {
                section.boundary.outerPolygon.edge.forEach((edge) => {
                    edge.curve.segment.forEach((segment) => {
                        const points = this.coordinates.applyOffsetToArray(segment.lineSegment.point);
                        const boundary = drawLaneMesh('CURB', points, this.colors?.colorMapping || colorMapping);
                        group.add(boundary);
                    });
                });
            });
            group.name = `Road-${id}`;
            this.roadMap[id] = group;
            this.scene.add(group);
        });
        this.removeOldLanes();
    }

    dispose() {
        Object.values(this.roadMap).forEach((item) => {
            disposeGroup(item);
            this.scene.remove(item);
        });
        this.roadMap = {};
        this.currentRoadIds = [];
    }

    removeOldLanes() {
        const needRemovedRoadIds = without(Object.keys(this.roadMap), ...this.currentRoadIds);
        // console.log(needRemovedRoadIds, '###');
        if (needRemovedRoadIds && needRemovedRoadIds.length) {
            needRemovedRoadIds.forEach((id) => {
                const removedGroup = this.roadMap[id];
                disposeGroup(removedGroup);
                this.scene.remove(removedGroup);
                delete this.roadMap[id];
            });
        }
    }
}
