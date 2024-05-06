import { without } from 'lodash';
import { drawPolygon } from '../../utils/polygon';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeMesh } from '../../utils/common';

export default class Junction {
    private meshs;

    private scene;

    private currentJunctionIds;

    private coordinates;

    private colors;

    constructor(scene, coordinates, colors?) {
        this.scene = scene;
        this.coordinates = coordinates;
        this.meshs = {};
        this.currentJunctionIds = [];
        this.colors = colors?.colorMapping || colorMapping;
    }

    drawJunctions(junctions) {
        this.currentJunctionIds = [];
        if (!junctions || junctions.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        junctions.forEach((junction) => {
            const id = junction.id.id;
            this.currentJunctionIds.push(id);
            if (this.meshs[id]) {
                return;
            }
            const points = this.coordinates.applyOffsetToArray(junction.polygon.point);
            const junctionMesh = drawPolygon(points, {
                color: this.colors.BLUE,
                linewidth: 1,
                zOffset: zOffset.junction,
                opacity: 1,
                matrixAutoUpdate: true,
            });
            junctionMesh.name = `junction${id}`;
            this.meshs[id] = junctionMesh;
            this.scene.add(junctionMesh);
        });
        this.removeOldJunctions();
    }

    dispose() {
        Object.values(this.meshs).forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.meshs = {};
        this.currentJunctionIds = [];
    }

    removeOldJunctions() {
        const needRemovedIds = without(Object.keys(this.meshs), ...this.currentJunctionIds);
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
