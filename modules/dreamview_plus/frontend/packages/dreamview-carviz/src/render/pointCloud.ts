import * as THREE from 'three';
import { pointCloudHeightColorMapping } from '../constant/common';
import { disposeMesh } from '../utils/common';

const MAX_POINTS = 200000;

export default class PointCloud {
    private pointCloudMesh;

    private scene;

    private option;

    private adc;

    constructor(scene, adc, option) {
        this.scene = scene;
        this.adc = adc;
        this.option = option;
        this.pointCloudMesh = null;
    }

    update(pointCloud) {
        this.dispose();
        if (!this.option.layerOption.Perception.pointCloud) {
            return;
        }
        if (!pointCloud.num || pointCloud.num.length % 3 !== 0) {
            console.warn('pointCloud length should be multiples of 3');
            return;
        }
        const adcMesh = this.adc?.adc || {};
        const { x = 0, y = 0 } = adcMesh.position;
        const heading = adcMesh?.rotation?.y || 0;

        const pointCloudSize = pointCloud.num.length / 3;
        const total = pointCloudSize < MAX_POINTS ? pointCloudSize : MAX_POINTS;
        let colorKey = 0.5;
        const positions = [];
        const colors = [];
        const geometry = new THREE.BufferGeometry();
        for (let i = 0; i < total; i += 1) {
            const x = pointCloud.num[i * 3];
            const y = pointCloud.num[i * 3 + 1];
            const z = pointCloud.num[i * 3 + 2];
            positions.push(x, y, z);

            if (z < 0.5) {
                colorKey = 0.5;
            } else if (z < 1.0) {
                colorKey = 1.0;
            } else if (z < 1.5) {
                colorKey = 1.5;
            } else if (z < 2.0) {
                colorKey = 2.0;
            } else if (z < 2.5) {
                colorKey = 2.5;
            } else if (z < 3.0) {
                colorKey = 3.0;
            } else {
                colorKey = 10.0;
            }
            const color = new THREE.Color(pointCloudHeightColorMapping[colorKey]);
            colors.push(color.r, color.g, color.b);
        }
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
        const material = new THREE.PointsMaterial({
            size: 0.25,
            vertexColors: true,
            transparent: true,
            opacity: 0.7,
        });
        const pointCloudMesh = new THREE.Points(geometry, material);
        pointCloudMesh.position.x = x;
        pointCloudMesh.position.y = y;
        pointCloudMesh.rotateZ(heading);
        this.pointCloudMesh = pointCloudMesh;
        this.scene.add(pointCloudMesh);
    }

    dispose() {
        disposeMesh(this.pointCloudMesh);
        this.scene.remove(this.pointCloudMesh);
        this.pointCloudMesh = null;
    }
}
