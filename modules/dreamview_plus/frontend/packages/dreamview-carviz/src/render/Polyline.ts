import { BufferGeometry, Mesh, Vector3 } from 'three';
import { MeshLine, MeshLineMaterial } from 'three.meshline';
import Point, { type IPointThreeContext } from './Point';
import Fan from './Fan';

export interface IPolylineThreeContext extends IPointThreeContext {
    polylineColor: number;
    fanColor: number;
    lineWidth: number;
    // 显示长度标签或者坐标标签
    label: 'length' | 'coordinate';
}

export default class Polyline {
    private points: Point[] = [];

    private fans: Fan[] = [];

    private line?: Mesh;

    private vertices: Vector3[];

    constructor(protected context: IPolylineThreeContext) {}

    public updateVertices(vertices: Vector3[], deleteLast = false) {
        if (vertices.length === 0) return this;
        this.vertices = vertices;
        this.createPoints();
        this.createLine();

        // 双击创建可能会多出一个点，需要删除
        if (deleteLast) {
            this.fans.pop()?.remove();
            this.points.pop()?.remove();
        }

        if (this.vertices.length >= 2) {
            this.createAngle();
        }

        return this;
    }

    private createPoints() {
        const { label } = this.context;
        for (let i = 0; i < this.vertices.length; i += 1) {
            if (!this.points[i]) {
                const coordinate = this.vertices[i].clone().add(new Vector3(0, 0, 0.01));

                this.points[i] = new Point(coordinate, 0, {
                    ...this.context,
                    marker: this.context.marker.clone(),
                });
            } else {
                this.points[i].updatePosition(this.vertices[i]);
            }
        }

        if (label === 'length' && this.vertices.length >= 2) {
            // 计算最后两个点的距离
            const lastPoint = this.points[this.points.length - 1];
            const secondLastPoint = this.points[this.points.length - 2];
            const length = lastPoint.position.distanceTo(secondLastPoint.position);
            lastPoint.setLengthLabelVisible(Number(length.toFixed(2)));
        }
        return this;
    }

    private createLine() {
        const { scene } = this.context;
        const line = new MeshLine();
        const geometry = new BufferGeometry().setFromPoints(this.vertices);
        line.setGeometry(geometry);
        if (this.line) {
            this.line.geometry = line.geometry;
            return this;
        }

        const material = new MeshLineMaterial({
            color: this.context.polylineColor || 0xffffff,
            lineWidth: this.context.lineWidth,
        });

        this.line = new Mesh(line, material);
        scene.add(this.line);
        return this;
    }

    private createAngle() {
        for (let i = 1; i < this.vertices.length - 1; i += 1) {
            if (!this.fans[i - 1]) {
                this.fans.push(
                    new Fan({
                        ...this.context,
                        fanColor: 0x1fcc4d,
                        borderWidth: 0.2,
                        borderColor: 0x1fcc4d,
                    })
                        .updatePoints(this.vertices[i - 1], this.vertices[i], this.vertices[i + 1])
                        .render(),
                );
            }
        }
        return this;
    }

    render() {
        const { scene, renderer, camera, CSS2DRenderer } = this.context;
        renderer.render(scene, camera);
        CSS2DRenderer.render(scene, camera);
        return this;
    }

    remove() {
        const { scene, renderer, camera } = this.context;
        this.points.forEach((point) => point.setLengthLabelVisible(0).remove());
        this.fans.forEach((fan) => fan.remove());
        if (this.line) {
            scene.remove(this.line);
            renderer.render(scene, camera);
        }

        return this;
    }
}
