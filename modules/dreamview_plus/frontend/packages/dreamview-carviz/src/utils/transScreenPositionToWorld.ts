import { Plane, Raycaster, Vector2, Vector3 } from 'three';
import { IThreeContext } from '../render/type';

// export default function transScreenPositionToWorld(
//     event: MouseEvent,
//     dom: HTMLElement,
//     camera: PerspectiveCamera,
// ) {
//     // 鼠标移动位置 和 3D位置转换
//     const pos = new Vector3();
//
//     const vecX = (event.clientX / dom.clientWidth) * 2 - 1;
//     const vecY = -(event.clientY / dom.clientHeight) * 2 + 1;
//     const vec = new Vector3(vecX, vecY, 0);
//
//     vec.unproject(camera);
//     vec.sub(camera.position).normalize();
//
//     const distance = -camera.position.z / vec.z;
//
//     return pos.copy(camera.position).add(vec.multiplyScalar(distance));
// }

interface ITransScreenPositionToWorldContext extends IThreeContext {
    raycaster: Raycaster;
}

function computeNormalizationPosition(x, y, context: ITransScreenPositionToWorldContext) {
    const { renderer } = context;
    const rect = renderer.domElement.getBoundingClientRect();
    return {
        x: ((x - rect.left) / rect.width) * 2 - 1,
        y: -((y - rect.top) / rect.height) * 2 + 1,
    };
}

export default function transScreenPositionToWorld(
    event: MouseEvent,
    context: ITransScreenPositionToWorldContext,
): Vector3 {
    const { camera, scene, raycaster } = context;
    const { x: nx, y: ny } = computeNormalizationPosition(event.clientX, event.clientY, context);
    raycaster.setFromCamera(new Vector2(nx, ny), camera);

    // 计算与射线相交的对象
    const intersects = raycaster.intersectObjects(scene.children, true);

    // 如果与物体有交点，返回第一个交点的坐标
    if (intersects.length > 0) {
        return intersects[0].point;
    }
    // 如果没有交点，则计算与xy平面的交点
    const plane = new Plane(new Vector3(0, 0, 1), 0);
    const planeIntersect = new Vector3();
    raycaster.ray.intersectPlane(plane, planeIntersect);

    return planeIntersect;
}
