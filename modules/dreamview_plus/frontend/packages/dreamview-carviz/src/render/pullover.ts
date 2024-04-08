import * as THREE from 'three';
import { drawPolygon } from '../utils/polygon';
import { colorMapping, zOffset } from '../constant/common';
import { drawCircle, disposeGroup } from '../utils/common';

export default class Pullover {
    private scene;

    private group;

    private colors;

    constructor(scene, colors?) {
        this.scene = scene;
        this.group = null;
        this.colors = colors?.colorMapping || colorMapping;
    }

    dispose() {
        disposeGroup(this.group);
        this.group = null;
    }

    drawPullover(pullover) {
        if (this.group) {
            this.dispose();
        }
        this.group = new THREE.Group();
        const { lengthFront, lengthBack, widthLeft, widthRight } = pullover;
        const points = [
            new THREE.Vector3(lengthFront, -widthLeft, 0),
            new THREE.Vector3(lengthFront, widthRight, 0),
            new THREE.Vector3(-lengthBack, widthRight, 0),
            new THREE.Vector3(-lengthBack, -widthLeft, 0),
        ];
        const polygonMesh = drawPolygon(points, {
            color: this.colors.PULLOVER,
            linewidth: 2,
            zOffset: zOffset.pullover,
            opacity: 1,
            matrixAutoUpdate: true,
        });
        this.group.add(polygonMesh);

        const material = new THREE.MeshBasicMaterial({
            color: this.colors.PULLOVER,
            transparent: true,
            opacity: 0.5,
        });
        const circleMesh = drawCircle(0.2, material);
        this.group.add(circleMesh);

        const arrow = new THREE.ArrowHelper(
            new THREE.Vector3(-1, 0, 0),
            new THREE.Vector3(0, 0, 0),
            1.5,
            this.colors.PULLOVER,
            0.5,
            0.5,
        );
        this.group.add(arrow);
        this.group.position.set(pullover.position.x, pullover.position.y, zOffset.pullover);
        this.group.rotation.set(0, 0, pullover.theta);
        this.scene.add(this.group);
    }
}
