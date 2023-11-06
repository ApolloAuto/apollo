import { CircleGeometry, DoubleSide, Mesh, MeshBasicMaterial, PlaneGeometry, Vector3 } from 'three';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import IcAnchorMarker from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcAnchorMarker.png';
import IcWaypointMarker from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcWaypointMarker.png';
import IcAnchorArrowMarker from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcAnchorArrowMarker.png';
import IcWaypointArrowMarker from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcWaypointArrowMarker.png';
import IcToolPointMarker from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcToolPointMarker.png';
import IcClose from '@dreamview/dreamview-carviz/assets/images/routing_editing/IcClose.png';
import { promiseTextureLoader } from '../../utils/promiseLoaders';
import { drawThickBandFromPoints } from '../../utils/common';

async function createMarker(radius: number, texture: string, rest?: { name?: string }, offset = [0, 0.084]) {
    const material = new MeshBasicMaterial({ map: await promiseTextureLoader(texture), transparent: true });
    material.map.offset.set(offset[0], offset[1]);
    const geometry = new CircleGeometry(radius, 32);
    const mesh = new Mesh(geometry, material);
    if (rest) {
        // 绑定自定义属性
        Object.keys(rest).forEach((key) => {
            mesh.userData[key] = rest[key];
        });
    }
    return mesh;
}

async function createArrow(width: number, height: number, texture: string): Promise<Mesh> {
    const geometry = new PlaneGeometry(width, height);
    geometry.rotateZ(-Math.PI / 2);
    geometry.translate(width / 2, 0, 0);
    const material = new MeshBasicMaterial({
        map: await promiseTextureLoader(texture),
        transparent: true,
        side: DoubleSide,
    });

    return new Mesh(geometry, material);
}

export async function createEditingAnchorMarker(radius: number) {
    return createMarker(radius, IcAnchorMarker);
}

export async function createEditingAnchorArrow(width: number, height: number) {
    return createArrow(width, height, IcAnchorArrowMarker);
}

export async function createPathwayAnchorMarker(radius: number) {
    return createMarker(radius, IcWaypointMarker);
}

export async function createPathwayAnchorArrow(width: number, height: number) {
    return createArrow(width, height, IcWaypointArrowMarker);
}

export async function createToolMarker(radius: number) {
    return createMarker(radius, IcToolPointMarker, null, [0, 0]);
}

export async function createCloseMarker(radius: number, rest?: { name?: string }) {
    return createMarker(radius, IcClose, rest, [0, 0]);
}

export function createToolSolidLine(vertices: Vector3[], lineWidth = 0.2, color = 0x3487f8) {
    return drawThickBandFromPoints(vertices, {
        color,
        lineWidth,
        opacity: 1,
    });
}

export const drawDashedLineFromPoints = (
    points: Vector3[],
    thickAttr: {
        color?: number;
        lineWidth?: number;
    },
) => {
    // 参数安全检查
    if (!Array.isArray(points) || points.length < 2) {
        console.warn('At least two points are required to draw a line.');
        return null;
    }

    if (typeof thickAttr !== 'object') {
        console.warn('Invalid attribute parameter provided.');
        return null;
    }

    const { color = 0xffffff, lineWidth = 0.5 } = thickAttr;

    const line = new MeshLine();
    line.setPoints(points);

    const distance = points[0].distanceTo(points[1]);

    // 确保分母不为0
    if (distance === 0) {
        console.warn('The provided points are too close or identical.');
        return null;
    }

    const dashArray = (1 / distance) * 0.5;

    const material = new MeshLineMaterial({
        color,
        lineWidth,
        dashArray,
    });

    return new Mesh(line.geometry, material);
};

export function createToolDashedLine(vertices: Vector3[], lineWidth = 0.2, color = 0x3487f8) {
    return drawDashedLineFromPoints(vertices, {
        color,
        lineWidth,
    });
}
