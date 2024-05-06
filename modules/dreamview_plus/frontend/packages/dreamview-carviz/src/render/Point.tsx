import { Mesh, AxesHelper, Vector3 } from 'three';
import { createRoot, Root } from 'react-dom/client';
import { CSS2DObject } from 'three/examples/jsm/renderers/CSS2DRenderer';
import { IThreeContext } from './type';
import PointLabel from '../labels/PointLabel';
import LengthLabel from '../labels/LengthLabel';
import type Coordinates from './coordinates';

export interface IPointThreeContext extends IThreeContext {
    coordinates: Coordinates;
    marker: Mesh;
    arrow?: Mesh;
}
export default class Point {
    public position: Vector3 = new Vector3();

    public direction = 0;

    public arrowVisible = false;

    public labelVisible = false;

    public lengthLabelVisible = false;

    private roots = new Map<Element, Root>();

    private pointLabel?: CSS2DObject;

    private lengthLabel?: CSS2DObject;

    constructor(position: Vector3, direction: number, protected context: IPointThreeContext) {
        this.position = position;
        this.direction = direction;
        const { scene, marker, arrow } = this.context;
        marker.position.copy(this.position);
        scene.add(marker);

        if (arrow && this.arrowVisible) {
            arrow.position.copy(this.position);
            arrow.position.z -= 0.1;
            arrow.rotation.z = this.direction;
            scene.add(arrow);
        } else if (arrow) {
            scene.remove(arrow);
        }
    }

    public setArrowVisible(visible: boolean) {
        const { scene, arrow } = this.context;
        this.arrowVisible = visible;
        if (arrow && visible) {
            scene.add(arrow);
        } else if (arrow) {
            scene.remove(arrow);
        }
        return this;
    }

    public setLabelVisible(visible: boolean) {
        const { marker, coordinates } = this.context;
        this.labelVisible = visible;

        const pointWithOffset = coordinates.applyOffset(this.position, true);
        this.pointLabel = this.createOrUpdateLabel({
            x: pointWithOffset.x.toFixed(2),
            y: pointWithOffset.y.toFixed(2),
        });
        if (this.pointLabel && visible) {
            marker.add(this.pointLabel);
        } else if (this.pointLabel) {
            marker.remove(this.pointLabel);
        }
        return this;
    }

    public setLengthLabelVisible(length: number) {
        const { marker } = this.context;
        this.lengthLabelVisible = length > 0;

        if (this.lengthLabelVisible) {
            if (this.lengthLabel) {
                this.createOrUpdateLengthLabel(length, this.lengthLabel.element);
            } else {
                this.lengthLabel = this.createOrUpdateLengthLabel(length);
                marker.add(this.lengthLabel);
            }
        } else {
            marker.remove(this.lengthLabel);
        }

        return this;
    }

    public updatePosition(position: Vector3) {
        this.position.copy(position);
        return this;
    }

    public updateDirection(direction: number) {
        this.direction = direction;
        this.setArrowVisible(true);
        return this;
    }

    private createOrUpdateLabel(coordinate, existingElement = null) {
        const labelComponent = <PointLabel coordinate={coordinate} />;

        if (existingElement) {
            let root = this.roots.get(existingElement);
            if (!root) {
                root = createRoot(existingElement);
                this.roots.set(existingElement, root);
            }
            root.render(labelComponent);
            this.pointLabel.position.set(0, 0, 0);
            return existingElement;
        }
        const element = document.createElement('div');
        const root = createRoot(element);
        this.roots.set(element, root);
        root.render(labelComponent);
        const label = new CSS2DObject(element);
        label.position.set(0, 0, 0);
        return label;
    }

    private createOrUpdateLengthLabel(length, existingElement = null) {
        const labelComponent = <LengthLabel length={length} />;

        if (existingElement) {
            let root = this.roots.get(existingElement);
            if (!root) {
                root = createRoot(existingElement);
                this.roots.set(existingElement, root);
            }
            root.render(labelComponent);
            this.lengthLabel.position.set(0, 0, 0);
            return existingElement;
        }
        const element = document.createElement('div');
        const root = createRoot(element);
        this.roots.set(element, root);
        root.render(labelComponent);
        const label = new CSS2DObject(element);
        label.position.set(0, 0, 0);
        return label;
    }

    addToScene() {
        const { scene, marker, arrow } = this.context;
        scene.add(marker);

        if (arrow && this.arrowVisible) {
            scene.add(arrow);
        }
        return this;
    }

    render() {
        // if (this.isRendered) return this;
        const { scene, renderer, camera, marker, arrow, CSS2DRenderer } = this.context;
        marker.position.copy(this.position);

        if (arrow && this.arrowVisible) {
            arrow.position.copy(this.position);
            arrow.position.z -= 0.1;
            arrow.rotation.z = this.direction;
        } else if (arrow) {
            scene.remove(arrow);
        }

        // 调试时使用坐标轴辅助线
        // const axesHelper = new AxesHelper(10);
        // axesHelper.position.copy(this.position);
        // scene.add(axesHelper);
        renderer.render(scene, camera);
        CSS2DRenderer.render(scene, camera);
        return this;
    }

    remove() {
        const { scene, renderer, camera, marker, arrow, CSS2DRenderer } = this.context;

        if (this.pointLabel) {
            this.pointLabel.element.remove();
            marker.remove(this.pointLabel);
        }

        if (this.lengthLabel) {
            this.lengthLabel.element.remove();
            marker.remove(this.lengthLabel);
        }

        marker.geometry.dispose();
        // @ts-ignore
        marker.material?.dispose();
        scene.remove(marker);
        if (arrow) {
            scene.remove(arrow);
        }
        renderer.render(scene, camera);
        CSS2DRenderer.render(scene, camera);
    }
}
