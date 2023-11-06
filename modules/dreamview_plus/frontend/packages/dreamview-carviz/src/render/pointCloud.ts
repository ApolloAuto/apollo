import * as THREE from 'three';
import Logger from '@dreamview/log';
import { pointCloudHeightColorMapping } from '../constant/common';
import { disposeMesh } from '../utils/common';

const logger = Logger.getInstance(`PointCloud-${Date.now()}`);

const MAX_POINTS = 200000;

export default class PointCloud {
    private pointCloudMesh;

    private scene;

    private option;

    private adc;

    private positionBuffer = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);

    private colorBuffer = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);

    private material = new THREE.PointsMaterial({
        size: 0.25,
        vertexColors: true,
        transparent: true,
        opacity: 0.7,
    });

    private geometry = new THREE.BufferGeometry();

    constructor(scene, adc, option) {
        this.scene = scene;
        this.adc = adc;
        this.option = option;
        this.pointCloudMesh = null;
        this.geometry.setAttribute('position', this.positionBuffer);
        this.geometry.setAttribute('color', this.colorBuffer);
    }

    update(pointCloud) {
        this.dispose();
        if (!this.option.layerOption.Perception.pointCloud || !pointCloud.num || pointCloud.num.length % 3 !== 0) {
            logger.warn('pointCloud length should be multiples of 3');
            return;
        }
        const adcMesh = this.adc?.adc || {};
        const x = adcMesh?.position?.x || 0;
        const y = adcMesh?.position?.y || 0;
        const heading = adcMesh?.rotation?.y || 0;

        const pointCloudSize = pointCloud.num.length / 3;
        const total = Math.min(pointCloudSize, MAX_POINTS);
        this.geometry.setDrawRange(0, total);

        for (let i = 0; i < total; i += 1) {
            const index = i * 3;
            const z = pointCloud.num[index + 2];

            const colorKey =
                // eslint-disable-next-line no-nested-ternary
                z < 0.5 ? 0.5 : z < 1.0 ? 1.0 : z < 1.5 ? 1.5 : z < 2.0 ? 2.0 : z < 2.5 ? 2.5 : z < 3.0 ? 3.0 : 10.0;

            const color = pointCloudHeightColorMapping[colorKey];

            this.positionBuffer.setXYZ(i, pointCloud.num[index], pointCloud.num[index + 1], z);
            this.colorBuffer.setXYZ(i, color.r, color.g, color.b);
        }

        this.positionBuffer.needsUpdate = true;
        this.colorBuffer.needsUpdate = true;
        const pointCloudMesh = new THREE.Points(this.geometry, this.material);
        pointCloudMesh.position.set(x, y, 0);
        pointCloudMesh.rotateZ(heading);
        this.pointCloudMesh = pointCloudMesh;
        this.scene.add(pointCloudMesh);
    }

    updateOffsetPosition() {
        if (!this.pointCloudMesh) {
            logger.warn('PointCloud mesh is not initialized');
            return;
        }

        const adcMesh = this.adc?.adc || {};
        const x = adcMesh?.position?.x || 0;
        const y = adcMesh?.position?.y || 0;
        const heading = adcMesh?.rotation?.y || 0;

        this.pointCloudMesh.position.set(x, y, 0);
        this.pointCloudMesh.rotation.set(0, 0, heading);
    }

    dispose() {
        disposeMesh(this.pointCloudMesh);
        this.scene.remove(this.pointCloudMesh);
        this.pointCloudMesh = null;
    }
}
