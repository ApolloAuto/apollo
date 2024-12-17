import * as THREE from 'three';
import { disposeGroup, drawCircle } from '../utils/common';
import { drawSegmentsFromPoints } from '../utils/line';

export default class Gps {
    private group;

    private scene;

    private adc;

    private option;

    private coordinates;

    constructor(scene, adc, option, coordinates) {
        this.group = null;
        this.scene = scene;
        this.option = option;
        this.adc = adc;
        this.coordinates = coordinates;
    }

    update(gps, adcInfo) {
        if (!this.adc?.vehicleParam) {
            return;
        }
        if (!this.coordinates.isInitialized()) {
            return;
        }
        if (!this.group) {
            this.group = new THREE.Group();
            const material = new THREE.MeshBasicMaterial({
                color: 0x006aff,
                transparent: true,
                opacity: 0.5,
            });
            const circle = drawCircle(0.2, material);
            const vehicleParam = this.adc.vehicleParam;
            const points = [
                new THREE.Vector3(vehicleParam.frontEdgeToCenter, -vehicleParam.leftEdgeToCenter, 0),
                new THREE.Vector3(vehicleParam.frontEdgeToCenter, vehicleParam.rightEdgeToCenter, 0),
                new THREE.Vector3(-vehicleParam.backEdgeToCenter, vehicleParam.rightEdgeToCenter, 0),
                new THREE.Vector3(-vehicleParam.backEdgeToCenter, -vehicleParam.leftEdgeToCenter, 0),
                new THREE.Vector3(vehicleParam.frontEdgeToCenter, -vehicleParam.leftEdgeToCenter, 0),
            ];
            const base = drawSegmentsFromPoints(points, {
                color: 0x006aff,
                linewidth: 2,
                zOffset: 0,
                opacity: 1,
                matrixAutoUpdate: true,
            });
            this.group.add(circle);
            this.group.add(base);
        }

        const { positionX, positionY } = gps;
        const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
        this.group.position.set(position.x, position.y, 0.01);
        this.group.visible = this.option.layerOption.Position.gps;
        this.group.rotation.set(0, 0, adcInfo?.heading || 0);
        this.scene.add(this.group);
    }

    dispose() {
        if (!this.group) {
            return;
        }
        disposeGroup(this.group);
        this.scene.remove(this.group);
        this.group = null;
    }
}
