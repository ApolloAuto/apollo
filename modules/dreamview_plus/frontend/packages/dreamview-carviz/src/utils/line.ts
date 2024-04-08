import * as THREE from 'three';
import { zOffset } from '../constant/common';

export function drawDashedLineFromPoints(points, lineAttr) {
    const {
        color = 0xff0000,
        linewidth = 1,
        dashSize = 4,
        gapSize = 2,
        zOffset = 0,
        opacity = 1,
        matrixAutoUpdate = true,
    } = lineAttr;
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineDashedMaterial({
        color,
        dashSize,
        linewidth,
        gapSize,
        transparent: true,
        opacity,
    });
    material.depthTest = true;
    material.transparent = true;
    // material.precision = 'highp';
    material.side = THREE.DoubleSide;
    // material.blending = THREE.LessDepth;
    const mesh = new THREE.Line(geometry, material);
    mesh.computeLineDistances();
    mesh.position.z = zOffset;
    mesh.matrixAutoUpdate = matrixAutoUpdate;
    if (!matrixAutoUpdate) {
        mesh.updateMatrix();
    }
    return mesh;
}

export function drawDashedLineFromPointsClone(points, material) {
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const mesh = new THREE.Line(geometry, material);
    mesh.computeLineDistances();
    return mesh;
}

export function drawSegmentsFromPointsClone(points, material) {
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const mesh = new THREE.Line(geometry, material);
    return mesh;
}

export function drawSegmentsFromPoints(points, lineAttr) {
    const { color = 0xff0000, linewidth = 1, zOffset = 0, opacity = 1, matrixAutoUpdate = true } = lineAttr;
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const material = new THREE.LineBasicMaterial({
        color,
        linewidth,
        transparent: true,
        opacity,
    });
    const mesh = new THREE.Line(geometry, material);
    mesh.position.z = zOffset;
    mesh.matrixAutoUpdate = matrixAutoUpdate;
    if (matrixAutoUpdate === false) {
        mesh.updateMatrix();
    }
    return mesh;
}

export function drawLaneMesh(laneType, points, colorMapping) {
    let left = null;
    let right = null;
    switch (laneType) {
        case 'DOTTED_YELLOW':
            return drawDashedLineFromPoints(points, {
                color: colorMapping.YELLOW,
                linewidth: 4,
                dashSize: 3,
                gapSize: 3,
                zOffset: zOffset.lane,
                opacity: 1,
                matrixAutoUpdate: false,
            });
        case 'DOTTED_WHITE':
            return drawDashedLineFromPoints(points, {
                color: colorMapping.WHITE,
                linewidth: 2,
                dashSize: 0.5,
                gapSize: 0.25,
                zOffset: zOffset.lane,
                opacity: 0.4,
                matrixAutoUpdate: false,
            });
        case 'SOLID_YELLOW':
            return drawSegmentsFromPoints(points, {
                color: colorMapping.YELLOW,
                linewidth: 3,
                zOffset: zOffset.lane,
                opacity: 1,
                matrixAutoUpdate: false,
            });
        case 'SOLID_WHITE':
            return drawSegmentsFromPoints(points, {
                color: colorMapping.WHITE,
                linewidth: 3,
                zOffset: zOffset.lane,
                opacity: 1,
                matrixAutoUpdate: false,
            });
        case 'DOUBLE_YELLOW':
            left = drawSegmentsFromPoints(points, {
                color: colorMapping.YELLOW,
                linewidth: 2,
                zOffset: zOffset.lane,
                opacity: 1,
                matrixAutoUpdate: false,
            });
            right = drawSegmentsFromPoints(
                points.map((point) => new THREE.Vector3(point.x + 0.3, point.y + 0.3, point.z)),
                {
                    color: colorMapping.YELLOW,
                    linewidth: 3,
                    zOffset: zOffset.lane,
                    opacity: 1,
                    matrixAutoUpdate: false,
                },
            );
            left.add(right);
            return left;
        case 'CURB':
            return drawSegmentsFromPoints(points, {
                color: colorMapping.CORAL,
                linewidth: 3,
                zOffset: zOffset.lane,
                opacity: 1,
                matrixAutoUpdate: false,
            });
        default:
            return drawSegmentsFromPoints(points, {
                color: colorMapping.DEFAULT,
                linewidth: 3,
                zOffset: zOffset.lane,
                opacity: 1,
                matrixAutoUpdate: false,
            });
    }
}
