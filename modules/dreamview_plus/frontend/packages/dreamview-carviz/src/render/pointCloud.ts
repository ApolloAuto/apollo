import * as THREE from 'three';
import Logger from '@dreamview/log';
import { DreamviewAnalysis, perfMonitor } from '@dreamview/dreamview-analysis';
import { pointCloudHeightColorMapping } from '../constant/common';
import { disposeMesh, getPointSize } from '../utils/common';

const logger = Logger.getInstance(`PointCloud-${Date.now()}`);

const MAX_POINTS = 200000;

export default class PointCloud {
    private pointCloudMesh;

    private scene;

    private option;

    private adc;

    private lastHeading;

    private colors;

    private meshFlag;

    constructor(scene, adc, option, colors?) {
        this.colors = colors?.pointCloudHeightColorMapping || pointCloudHeightColorMapping;
        this.scene = scene;
        this.adc = adc;
        this.option = option;
        this.lastHeading = 0;
        this.meshFlag = false;
        this.pointCloudMesh = null;
        this.initPointCloudMesh();
    }

    initPointCloudMesh() {
        const positionBuffer = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
        const colorBuffer = new THREE.BufferAttribute(new Float32Array(MAX_POINTS * 3), 3);
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', positionBuffer);
        geometry.setAttribute('color', colorBuffer);
        geometry.setDrawRange(0, 0);
        geometry.getAttribute('position').needsUpdate = true;

        const material = new THREE.PointsMaterial({
            size: 0.05,
            vertexColors: true,
            transparent: true,
            opacity: 0.7,
        });

        const pointCloudMesh = new THREE.Points(geometry, material);
        this.pointCloudMesh = pointCloudMesh;
    }

    update(pointCloud) {
        perfMonitor.mark('pointCloudUpdateStart');
        if (!this.option.layerOption.Perception.pointCloud || !pointCloud.num || pointCloud.num.length % 3 !== 0) {
            logger.warn('pointCloud length should be multiples of 3');
            return;
        }
        const adcMesh = this.adc?.adc || {};
        const x = adcMesh?.position?.x || 0;
        const y = adcMesh?.position?.y || 0;
        const heading = adcMesh?.rotation?.y || 0;
        const pointCloudSize = pointCloud.num.length / 3;
        DreamviewAnalysis.logData(
            'pointCloud',
            {
                pointCloudSize,
            },
            {
                useStatistics: {
                    useMax: true,
                },
            },
        );
        const total = Math.min(pointCloudSize, MAX_POINTS);

        this.pointCloudMesh.geometry.setDrawRange(0, total);
        this.pointCloudMesh.material.setValues({
            size: getPointSize(pointCloudSize),
        });

        for (let i = 0; i < total; i += 1) {
            const index = i * 3;
            const z = pointCloud.num[index + 2];

            const colorKey =
                // eslint-disable-next-line no-nested-ternary
                z < 0.5 ? 0.5 : z < 1.0 ? 1.0 : z < 1.5 ? 1.5 : z < 2.0 ? 2.0 : z < 2.5 ? 2.5 : z < 3.0 ? 3.0 : 10.0;

            const color = this.colors[colorKey];

            const positions = this.pointCloudMesh.geometry.attributes.position;
            const colorsArray = this.pointCloudMesh.geometry.attributes.color;

            positions.setXYZ(i, pointCloud.num[index], pointCloud.num[index + 1], z);
            colorsArray.setXYZ(i, color.r, color.g, color.b);
        }
        this.pointCloudMesh.position.set(x, y, 0);
        this.pointCloudMesh.rotateZ(heading - this.lastHeading);
        this.lastHeading = heading;
        this.pointCloudMesh.geometry.attributes.position.needsUpdate = true;
        this.pointCloudMesh.geometry.attributes.color.needsUpdate = true;

        if (!this.meshFlag) {
            this.meshFlag = true;
            this.scene.add(this.pointCloudMesh);
        }
        perfMonitor.mark('pointCloudUpdateEnd');
        perfMonitor.measure('pointCloudUpdate', 'pointCloudUpdateStart', 'pointCloudUpdateEnd');
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
        // disposeMesh(this.pointCloudMesh);
        // this.scene.remove(this.pointCloudMesh);
        // this.pointCloudMesh = null;
    }

    disposeLastFrame() {
        this.meshFlag = false;
        this.scene.remove(this.pointCloudMesh);
    }
}
