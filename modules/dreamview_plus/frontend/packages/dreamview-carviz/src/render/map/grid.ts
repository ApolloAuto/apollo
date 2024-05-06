import * as THREE from 'three';
import { disposeMesh } from '../../utils/common';

export default class Grid {
    private mesh;

    private scene;

    private size;

    constructor(scene) {
        this.mesh = null;
        this.scene = scene;
        this.size = 0;
    }

    drawGrid(gidAttr, position) {
        const { size, divisions, colorCenterLine = 0x000000, colorGrid = 0x000000 } = gidAttr;
        if (size === this.size && this.mesh) {
            this.mesh.position.set(position.x, position.y, 0);
            return;
        }
        this.size = size;
        this.dispose();
        const gridHelper = new THREE.GridHelper(size, divisions, colorCenterLine, colorGrid);
        gridHelper.rotateX(Math.PI / 2);
        gridHelper.name = 'gridHelper';
        (gridHelper.material as THREE.LineBasicMaterial).opacity = 0.3;
        (gridHelper.material as THREE.LineBasicMaterial).transparent = true;
        gridHelper.position.set(position.x, position.y, 0);
        this.scene.add(gridHelper);
        this.mesh = gridHelper;
    }

    dispose() {
        disposeMesh(this.mesh);
        this.scene.remove(this.mesh);
        this.mesh = null;
    }
}
