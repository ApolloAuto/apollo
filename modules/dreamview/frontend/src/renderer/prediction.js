import * as THREE from "three";

import STORE from "store";

import { DEFAULT_COLOR, ObstacleColorMapping } from "renderer/obstacles.js";
import { drawCircle, drawSegmentsFromPoints } from "utils/draw";

const _ = require('lodash');
const majorThickness = 3;

export default class Prediction {
    constructor() {
        this.predLines = []; // Prediction lines to indicate direction
        this.predCircles = []; // Prediction circles to indicate speed
    }

    update(world, coordinates, scene) {
        // Clear out the preiction lines/circles from last frame.
        this.predLines.forEach(p => {
            scene.remove(p);
            p.geometry.dispose();
            p.material.dispose();
        });
        this.predLines = [];

        this.predCircles.forEach(c => {
            scene.remove(c);
            c.geometry.dispose();
            c.material.dispose();
        });
        this.predCircles = [];

        if (!STORE.options.showPredictionMajor && !STORE.options.showPredictionMinor) {
            return;
        }

        if (_.isEmpty(world.object)) {
            return;
        }
        world.object.forEach(obj => {
            const predictionLineColor = ObstacleColorMapping[obj.type] || DEFAULT_COLOR;
            const predictions = obj.prediction;
            if (_.isEmpty(predictions)) {
                return;
            }

            if (!STORE.options['showObstacles' + _.upperFirst(_.camelCase(obj.type))]) {
                return;
            }

            // Take the prediction line with highest probability as major, others as minor.
            _.sortBy(predictions, o => o.probablity);
            const predictionMajor = predictions[predictions.length - 1];
            const predictionMinor = predictions.slice(0, predictions.length - 1);

            if (STORE.options.showPredictionMajor) {
                const predictedTraj = coordinates.applyOffsetToArray(
                    predictionMajor.predictedTrajectory);
                const mesh = drawSegmentsFromPoints(predictedTraj,
                    predictionLineColor, majorThickness, 6);
                this.predLines.push(mesh);
                scene.add(mesh);

                // Downsampling points to draw circles
                const downsamplingRatio = Math.ceil(predictedTraj.length / 3);
                for (let j = 0; j < predictedTraj.length; j += downsamplingRatio) {
                    const circleMesh = this.getPredCircle();
                    circleMesh.position.set(predictedTraj[j].x, predictedTraj[j].y, 0.24);
                    circleMesh.material.color.setHex(predictionLineColor);
                    scene.add(circleMesh);
                }
            }

            let minorThickness = 2.3;
            if (STORE.options.showPredictionMinor) {
                predictionMinor.forEach(prediction => {
                    const mesh = drawSegmentsFromPoints(
                            coordinates.applyOffsetToArray(prediction.predictedTrajectory),
                            predictionLineColor, minorThickness, 6);
                    this.predLines.push(mesh);
                    scene.add(mesh);

                    // keep thickness the same trajectories with low probabilities
                    if (minorThickness > 0.2) {
                        minorThickness -= 0.7;
                    }
                });
            }
        });
    }

    getPredCircle() {
        const material = new THREE.MeshBasicMaterial({
            color: 0xffffff,
            transparent: false,
            opacity: 0.5
        });
        const circleMesh = drawCircle(0.2, material);
        this.predCircles.push(circleMesh);
        return circleMesh;
    }
}
