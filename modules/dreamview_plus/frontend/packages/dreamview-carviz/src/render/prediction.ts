import { sortBy } from 'lodash';
import * as THREE from 'three';
import { disposeMesh, drawEllipseGeometry, drawCircle } from '../utils/common';
import { drawSegmentsFromPoints } from '../utils/line';
import { obstacleColorMapping, zOffset } from '../constant/common';

const EPSILON = 1e-3;

export default class Prediction {
    private majorMeshs;

    private minorMeshs;

    private guassMeshs;

    private scene;

    private option;

    private coordinates;

    private colors;

    constructor(scene, option, coordinates, colors?) {
        this.colors = colors?.obstacleColorMapping || obstacleColorMapping;
        this.majorMeshs = [];
        this.minorMeshs = [];
        this.guassMeshs = [];
        this.scene = scene;
        this.option = option;
        this.coordinates = coordinates;
    }

    update(obstacles) {
        const { majorPredictionLine, minorPredictionLine } = this.option.layerOption.Prediction;
        this.dispose();
        if (majorPredictionLine) {
            this.updateMajorPrediction(obstacles);
        }
        if (minorPredictionLine) {
            this.updateMinorPrediction(obstacles);
        }
    }

    updateMajorPrediction(obstacles) {
        if (!this.coordinates.isInitialized()) {
            return;
        }
        for (let i = 0; i < obstacles.length; i += 1) {
            const obstacle = obstacles[i];
            const predictions = obstacle.prediction;
            const color = this.colors[obstacle.type] || this.colors.DEFAULT;
            if (!predictions || predictions.length === 0) {
                continue;
            }

            sortBy(predictions, (o) => o.probability);
            const majorPrediction = predictions[predictions.length - 1];
            const points = this.coordinates.applyOffsetToArray(majorPrediction.predictedTrajectory);
            const mesh = drawSegmentsFromPoints(points, {
                color,
                linewidth: 3,
                zOffset: zOffset.prediction,
                opacity: 1,
                matrixAutoUpdate: true,
            });
            this.majorMeshs.push(mesh);
            this.scene.add(mesh);

            for (let j = 0; j < points.length; j += 1) {
                const point = points[j];
                const circleMesh = this.drawCircle();
                circleMesh.position.set(point.x, point.y, zOffset.prediction);
                circleMesh.material.color.setHex(color);
                this.majorMeshs.push(circleMesh);
                this.scene.add(circleMesh);

                if (this.option.layerOption.Prediction.gaussianInfo) {
                    const gaussianMesh = this.drawGaussian(point.gaussianInfo, color);
                    if (gaussianMesh) {
                        this.guassMeshs.push(gaussianMesh);
                        this.scene.add(gaussianMesh);
                    }
                }
            }
        }
    }

    updateMinorPrediction(obstacles) {
        if (!this.coordinates.isInitialized()) {
            return;
        }
        for (let i = 0; i < obstacles.length; i += 1) {
            const obstacle = obstacles[i];
            const predictions = obstacle.prediction;
            const color = this.colors[obstacle.type] || this.colors.DEFAULT;
            if (!predictions || predictions.length === 0) {
                continue;
            }

            sortBy(predictions, (o) => o.probability);
            const minorPrediction = predictions.slice(0, predictions.length - 1);
            if (minorPrediction && minorPrediction.length !== 0) {
                for (let j = 0; j < minorPrediction.length; j += 1) {
                    const traj = minorPrediction[j];
                    const points = traj.predictedTrajectory;
                    if (points && points.length !== 0) {
                        const mesh = drawSegmentsFromPoints(points, {
                            color,
                            linewidth: 2.3,
                            zOffset: zOffset.prediction,
                            opacity: 1,
                            matrixAutoUpdate: true,
                        });
                        this.minorMeshs.push(mesh);
                        this.scene.add(mesh);

                        if (this.option.layerOption.Prediction.gaussianInfo) {
                            for (let k = 0; k < points.length; k += 1) {
                                const point = points[k];
                                const gaussianMesh = this.drawGaussian(point.gaussianInfo, color);
                                if (gaussianMesh) {
                                    this.guassMeshs.push(gaussianMesh);
                                    this.scene.add(gaussianMesh);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    disposeMajorMeshs() {
        this.majorMeshs.forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.majorMeshs = [];
    }

    disposeMinorMeshs() {
        this.minorMeshs.forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.minorMeshs = [];
    }

    disposeGaussMeshs() {
        this.guassMeshs.forEach((mesh) => {
            disposeMesh(mesh);
            this.scene.remove(mesh);
        });
        this.guassMeshs = [];
    }

    drawGaussian(gaussian, color) {
        if (gaussian && gaussian.ellipseA > EPSILON && gaussian.ellipseB > EPSILON) {
            const material = new THREE.MeshBasicMaterial({
                color,
                transparent: true,
                opacity: 0.5,
            });
            const geometry = drawEllipseGeometry(gaussian.ellipseA, gaussian.ellipseB);
            const mesh = new THREE.Mesh(geometry, material);
            return mesh;
        }
        return null;
    }

    drawCircle() {
        const material = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            transparent: true,
            opacity: 0.5,
        });
        const circleMesh = drawCircle(0.2, material);
        return circleMesh;
    }

    dispose() {
        this.disposeMajorMeshs();
        this.disposeMinorMeshs();
        this.disposeGaussMeshs();
    }
}
