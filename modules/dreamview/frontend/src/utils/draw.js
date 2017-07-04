import * as THREE from "three";
import ThreeLine2D from "three-line-2d";
import ThreeLine2DBasicShader from "three-line-2d/shaders/basic";

const DELTA_Z_OFFSET = 0.04;
const Line = ThreeLine2D(THREE);
const BasicShader = ThreeLine2DBasicShader(THREE);

export function addOffsetZ(mesh, value) {
    if (value) {
        const zOffset = value * DELTA_Z_OFFSET;
        mesh.position.z += zOffset;
    }
}

export function drawDashedLineFromPoints(
    points, color = 0xff0000, linewidth = 1, dashSize = 4,
    gapSize = 2, zOffset = 0, matrixAutoUpdate = true) {

    const geometry = new THREE.Geometry();
    geometry.vertices = points;
    geometry.computeLineDistances();
    const material = new THREE.LineDashedMaterial({
        color: color,
        dashSize: dashSize,
        linewidth: linewidth,
        gapSize: gapSize
    });
    const mesh = new THREE.Line(geometry, material);
    addOffsetZ(mesh, zOffset);
    mesh.matrixAutoUpdate = matrixAutoUpdate;
    if (!matrixAutoUpdate) {
        mesh.updateMatrix();
    }
    return mesh;
}

export function drawCircle(radius, material, segments = 32) {
    const geometry = new THREE.CircleGeometry(radius, segments);
    const circleMesh = new THREE.Mesh(geometry, material);
    return circleMesh;
}

export function drawThickBandFromPoints(
    points, thickness = 0.5, color = 0xffffff, opacity = 1, zOffset = 0) {
    // const quality = 5;
    // const curve = bezier(points.map(p => [p.x, p.y]), quality);

    const geometry = Line(points.map(p => [p.x, p.y]));

    const material = new THREE.ShaderMaterial(BasicShader({
        side: THREE.DoubleSide,
        diffuse: color,
        thickness: thickness,
        opacity: opacity,
        transparent: true,
    }));
    const mesh = new THREE.Mesh(geometry, material);
    addOffsetZ(mesh, zOffset);
    return mesh;
}
