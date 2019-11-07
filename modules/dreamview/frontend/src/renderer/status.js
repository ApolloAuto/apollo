import * as THREE from "three";
import STORE from "store";
import { drawSegmentsFromPoints, drawCircle, drawArrow, disposeMeshGroup } from "utils/draw";

function drawPullOverBox({ lengthFront, lengthBack, widthLeft, widthRight }) {
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
        this.pullOverBox = null;
    }

    update(planningData, coordinates, scene) {
        // Dispose old status
        if (this.pullOverBox) {
            disposeMeshGroup(this.pullOverBox);
            scene.remove(this.pullOverBox);
            this.pullOverBox = null;
        }

        if (!planningData) {
            return;
        }

        // Draw pull over status
        if (STORE.options.customizedToggles.get('pullOver')) {
            const { pullOver } = planningData;
            if (pullOver) {
                this.pullOverBox = drawPullOverBox(pullOver);

                const position = coordinates.applyOffset({
                    x: pullOver.position.x,
                    y: pullOver.position.y,
                    z: 0.3,
                });
                this.pullOverBox.position.set(position.x, position.y, position.z);
                this.pullOverBox.rotation.set(0, 0, pullOver.theta);
                scene.add(this.pullOverBox);
            }
        }
    }
}