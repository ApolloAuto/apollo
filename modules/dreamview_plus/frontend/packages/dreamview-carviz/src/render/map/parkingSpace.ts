import { without } from 'lodash';
import { drawPolygon } from '../../utils/polygon';
import { colorMapping, zOffset } from '../../constant/common';
import { disposeMesh } from '../../utils/common';

export default class ParkingSpace {
    private meshs;

    private ids;

    private scene;

    private text;

    private option;

    private currentIds;

    private coordinates;

    private colors;

    constructor(scene, text, option, coordinates, colors?) {
        this.scene = scene;
        this.coordinates = coordinates;
        this.option = option;
        this.meshs = {};
        this.ids = {};
        this.text = text;
        this.currentIds = [];
        this.colors = colors?.colorMapping || colorMapping;
    }

    drawParkingSpaces(parkingSpaces) {
        this.currentIds = [];
        if (!parkingSpaces || parkingSpaces.length === 0) {
            this.dispose();
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        parkingSpaces.forEach((parkingSpace) => {
            const id = parkingSpace.id.id;
            this.currentIds.push(id);
            if (this.meshs[id]) {
                return;
            }
            const points = this.coordinates.applyOffsetToArray(parkingSpace.polygon.point);
            const mesh = drawPolygon(points, {
                color: this.colors.YELLOW,
                linewidth: 2,
                zOffset: zOffset.parkingSpace,
                opacity: 1,
                matrixAutoUpdate: true,
            });
            mesh.name = "ParkingSpace";
            mesh.userData = {color: this.colors.YELLOW,
                        selectedColor: 'rgb(247, 86, 96)'};
            this.meshs[id] = mesh;
            this.scene.add(mesh);

            if (this.option.layerOption.Map.parkingSpaceId) {
                this.drawParkingSpaceId(parkingSpace);
            }
        });
        this.removeOldGroups();
    }

    drawParkingSpaceId(parkingSpace) {
        const id = parkingSpace.id.id;
        if (this.ids[id]) {
            return;
        }
        const points = this.coordinates.applyOffsetToArray(parkingSpace?.polygon?.point);
        if (points && points.length >= 3) {
            const point1 = points[0];
            const point2 = points[1];
            const point3 = points[2];
            const position = {
                x: (point1.x + point3.x) / 2,
                y: (point1.y + point3.y) / 2,
                z: 0.04,
            };
            const rotationZ = Math.atan2(point2.y - point1.y, point2.x - point1.x);
            const text = this.text.drawText(id, this.colors.WHITE, position);
            text.rotation.z = rotationZ;
            this.ids[id] = text;
            this.scene.add(text);
        }
    }

    dispose() {
        this.disposeParkingSpaceIds();
        this.disposeParkingSpaces();
    }

    disposeParkingSpaces() {
        Object.values(this.meshs).forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.meshs = {};
    }

    disposeParkingSpaceIds() {
        Object.values(this.ids).forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.ids = {};
        this.currentIds = [];
    }

    removeOldGroups() {
        const needRemovedIds = without(Object.keys(this.meshs), ...this.currentIds);
        if (needRemovedIds && needRemovedIds.length) {
            needRemovedIds.forEach((id) => {
                const removedMesh = this.meshs[id];
                disposeMesh(removedMesh);
                this.scene.remove(removedMesh);
                delete this.meshs[id];

                const text = this.ids[id];
                disposeMesh(text);
                this.scene.remove(text);
                delete this.ids[id];
            });
        }
    }
}
