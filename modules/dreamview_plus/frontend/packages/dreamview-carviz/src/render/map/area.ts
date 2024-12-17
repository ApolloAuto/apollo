import { without } from 'lodash';
import { drawPolygon } from '../../utils/polygon';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeMesh } from '../../utils/common';

export default class Area {
    private meshs;

    private scene;

    private currentAreaIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.scene = scene;
        this.coordinates = coordinates;
        this.meshs = {};
        this.currentAreaIds = [];
        this.colors = colors?.colorMapping || colorMapping;
    }

    drawAreas(areas) {
        this.currentAreaIds = [];
        if (!areas || areas.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        areas.forEach((area) => {
            const id = area.id.id;
            this.currentAreaIds.push(id);
            if (this.meshs[id]) {
                return;
            }
            const points = this.coordinates.applyOffsetToArray(area.polygon.point);
            const areaMesh = drawPolygon(points, {
                color: this.colors.DEEP_RED,
                linewidth: 1,
                zOffset: zOffset.area,
                opacity: 1,
                matrixAutoUpdate: true,
            });
            areaMesh.name = `area${id}`;
            this.meshs[id] = areaMesh;
            this.scene.add(areaMesh);
        });
        this.removeOldAreas();
    }

    dispose() {
        Object.values(this.meshs).forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.meshs = {};
        this.currentAreaIds = [];
    }

    removeOldAreas() {
        const needRemovedIds = without(Object.keys(this.meshs), ...this.currentAreaIds);
        if (needRemovedIds && needRemovedIds.length) {
            needRemovedIds.forEach((id) => {
                const mesh = this.meshs[id];
                disposeMesh(mesh);
                this.scene.remove(mesh);
                delete this.meshs[id];
            });
        }
    }
}
