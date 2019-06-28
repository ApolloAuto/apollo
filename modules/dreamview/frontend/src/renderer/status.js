import * as THREE from "three";
import STORE from "store";
import { drawSegmentsFromPoints, drawCircle, drawArrow } from "utils/draw";

function getPullOverStatus({ lengthFront, lengthBack, widthLeft, widthRight }) {
    const pullOverStatus = new THREE.Group();
    const color = 0x006AFF;
    const polygon = drawSegmentsFromPoints(
        [
            new THREE.Vector3(lengthFront, -widthLeft, 0),
            new THREE.Vector3(lengthFront, widthRight, 0),
            new THREE.Vector3(-lengthBack, widthRight, 0),
            new THREE.Vector3(-lengthBack, -widthLeft, 0),
            new THREE.Vector3(lengthFront, -widthLeft, 0),
        ],
        color,
        2,
        5,
    );
    pullOverStatus.add(polygon);

    const material = new THREE.MeshBasicMaterial({
        color,
        transparent: false,
        opacity: 0.5
    });
    const circle = drawCircle(0.2, material);
    pullOverStatus.add(circle);

    const heading = drawArrow(1.5, 2, 0.5, 0.5, color);
    heading.rotation.set(0, 0, -Math.PI / 2);
    pullOverStatus.add(heading);

    return pullOverStatus;
}

export default class PlanningStatus {
    constructor() {
        this.pullOverStatus = null;
    }

    update(planningData, coordinates, scene) {
        // Draw pull over status
        if (planningData) {
            const { pullOverStatus } = planningData;
            if (!STORE.options.customizedToggles.get('pullOverStatus')) {
                if (this.pullOverStatus) {
                    this.pullOverStatus.traverse((child) => {
                        child.visible = false;
                    });
                }
            } else {
                if (this.pullOverStatus) {
                    this.pullOverStatus.traverse((child) => {
                        if (child.geometry !== undefined) {
                            child.geometry.dispose();
                            child.material.dispose();
                        }
                    });
                    scene.remove(this.pullOverStatus);
                    this.pullOverStatus = null;
                }
                if (pullOverStatus) {
                    this.pullOverStatus = getPullOverStatus(pullOverStatus);

                    const position = coordinates.applyOffset({
                        x: pullOverStatus.position.x,
                        y: pullOverStatus.position.y,
                        z: 0.3,
                    });
                    this.pullOverStatus.position.set(position.x, position.y, position.z);
                    this.pullOverStatus.rotation.set(0, 0, pullOverStatus.theta);
                    scene.add(this.pullOverStatus);
                }
            }
        }
    }
}