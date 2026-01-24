import * as THREE from 'three';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import { isNaN } from 'lodash';
import { drawSegmentsFromPoints } from './line';

export const coordinatesSame = (coord1, coord2): boolean =>
    coord1.x === coord2.x && coord1.y === coord2.y && coord1.z === coord2.z;

export const getObjectBox3 = (object): THREE.Box3 => {
    const box = new THREE.Box3();
    box.expandByObject(object);
    return box;
};
export const getObjectLengthAndWidth = (object) => {
    const box = new THREE.Box3();
    box.expandByObject(object);
    const length = box.max.x - box.min.x;
    const width = box.max.y - box.min.y;
    return [length, width];
};

export const disposeMesh = (mesh) => {
    mesh?.geometry?.dispose();
    mesh?.material?.dispose();
};

export const disposeGroup = (group) => {
    group.traverse((child) => {
        disposeMesh(child);
    });
};

export const drawCircle = (radius, material, segments = 32) => {
    const geometry = new THREE.CircleGeometry(radius, segments);
    const circleMesh = new THREE.Mesh(geometry, material);
    return circleMesh;
};

export const drawArrow = (color, length = 1.5, conelength = 0.5, conewidth = 0.5) => {
    const end = new THREE.Vector3(length, 0, 0);
    const begin = new THREE.Vector3(0, 0, 0);
    const top = new THREE.Vector3(length - conewidth, conelength / 2, 0);
    const bottom = new THREE.Vector3(length - conewidth, -conelength / 2, 0);
    const points = [begin, end, top, end, bottom];
    const arrow = drawSegmentsFromPoints(points, {
        color,
        linewidth: 1,
        zOffset: 1,
        opacity: 1,
        matrixAutoUpdate: true,
    });
    return arrow;
};

export const drawImge = (img, width, height) => {
    const textLoader = new THREE.TextureLoader();
    const material = new THREE.MeshBasicMaterial({
        map: textLoader.load(img),
        transparent: true,
        depthWrite: false,
        side: THREE.DoubleSide,
    });
    const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height), material);
    return mesh;
};

export const addOutLineToObject = (object, color) => {
    const ouline = new THREE.LineSegments(
        new THREE.EdgesGeometry(object.geometry),
        new THREE.LineBasicMaterial({
            color,
            shadowSide: THREE.DoubleSide,
            depthTest: false,
        }),
    );
    object.add(ouline);
};

export const drawSolidBox = (x, y, z, color) => {
    const geometry = new THREE.BoxGeometry(x, y, z);
    const material = new THREE.MeshBasicMaterial({
        color,
        transparent: true,
        opacity: 0.8,
    });
    const box = new THREE.Mesh(geometry, material);
    addOutLineToObject(box, color);
    return box;
};

export const drawBox = (x, y, z, color) => {
    const geometry = new THREE.BoxGeometry(x, y, z);
    const material = new THREE.MeshBasicMaterial({ color });
    const box = new THREE.BoxHelper(new THREE.Mesh(geometry, material));
    box.material.color.set(color);
    return box;
};

export const drawDashedBox = (x, y, z, color) => {
    const geometry = new THREE.BoxGeometry(x, y, z);
    const edge = new THREE.EdgesGeometry(geometry);
    const dashedLine = new THREE.LineSegments(
        edge,
        new THREE.LineDashedMaterial({
            color,
            dashSize: 0.1,
            gapSize: 0.1,
        }),
    );
    dashedLine.computeLineDistances();
    return dashedLine;
};

export const drawThickBandFromPoints = (points, thickAttr) => {
    const { color = 0xffffff, opacity = 1, lineWidth = 0.5 } = thickAttr;
    if (!points || points.length === 0) {
        return null;
    }
    const geometry = new THREE.BufferGeometry().setFromPoints(points);
    const line = new MeshLine();
    line.setGeometry(geometry);
    const material = new MeshLineMaterial({
        color,
        lineWidth,
        opacity,
    });
    material.depthTest = true;
    material.transparent = true;
    material.side = THREE.DoubleSide;
    return new THREE.Mesh(line.geometry, material);
};

export const drawShapeFromPoints = (points, color) => {
    const shape = new THREE.Shape();
    shape.setFromPoints(points);
    const geometry = new THREE.ShapeGeometry(shape);
    const material = new THREE.MeshBasicMaterial({
        color,
    });
    const mesh = new THREE.Mesh(geometry, material);
    return mesh;
};

export const drawEllipseGeometry = (aRadius, bRadius) => {
    const path = new THREE.Shape();
    path.absellipse(0, 0, aRadius, bRadius, 0, Math.PI * 2, false, 0);
    const geometry = new THREE.ShapeGeometry(path);
    return geometry;
};

export function areVerticesValid(vertices) {
    for (let i = 0; i < vertices.length; i += 1) {
        const vert = vertices[i];
        if (isNaN(vert.x) || isNaN(vert.y) || isNaN(vert.z)) {
            return false;
        }
    }
    return true;
}

export function compareLineSame(oldLine, newLine) {
    return oldLine.every((item, index) => {
        return item.x === newLine[index].x && item.y === newLine[index].y && item.z === newLine[index].z;
    });
}

// 根据点云数量修改点云绘制点的大小
export function getPointSize(pointCount: number) {
    if (pointCount < 3000) {
        return 1;
    }
    return 0.05;
}
