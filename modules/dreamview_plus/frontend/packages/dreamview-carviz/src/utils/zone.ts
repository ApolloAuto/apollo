import * as THREE from 'three';
import { coordinatesSame } from './common';
import { colorMapping } from '../constant/common';

export const drawZone = (points, zoneAttr) => {
    if (points.length < 3) {
        throw new Error('there are less than 3 points, the zone cannot be drawn');
    }

    const { color = colorMapping.WHITE, zOffset = 0, opacity = 1, matrixAutoUpdate = true } = zoneAttr;

    const length = points.length;
    if (!coordinatesSame(points[0], points[length - 1])) {
        points.push(points[0]);
    }

    const shape = new THREE.Shape(points);
    const geometry = new THREE.ShapeGeometry(shape);
    const material = new THREE.MeshBasicMaterial({
        color,
        opacity,
        transparent: true,
    });
    const zoneMesh = new THREE.Mesh(geometry, material);
    zoneMesh.position.z = zOffset;

    if (!matrixAutoUpdate) {
        zoneMesh.updateMatrix();
    }
    return zoneMesh;
};
