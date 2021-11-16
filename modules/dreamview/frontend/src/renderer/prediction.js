import * as THREE from 'three';

import STORE from 'store';

import { DEFAULT_COLOR, ObstacleColorMapping } from 'renderer/obstacles.js';
import {
  drawCircle, drawEllipse, drawSegmentsFromPoints, disposeMesh,
} from 'utils/draw';

const _ = require('lodash');

const majorThickness = 3;

const EPSILON = 1e-3;

export default class Prediction {
  constructor() {
    this.predLines = []; // Prediction lines to indicate direction
    this.predCircles = []; // Prediction circles to indicate speed
    this.predGaussian = []; // Prediction ellipse to visualize gaussian
  }

  disposeMeshes(scene) {
    // Clear out the prediction lines/circles from last frame.
    this.predLines.forEach((p) => {
      scene.remove(p);
      disposeMesh(p);
    });
    this.predLines = [];

    this.predCircles.forEach((c) => {
      scene.remove(c);
      disposeMesh(c);
    });
    this.predCircles = [];

    this.predGaussian.forEach((g) => {
      scene.remove(g);
      disposeMesh(g);
    });
    this.predGaussian = [];
  }

  update(world, coordinates, scene) {
    this.disposeMeshes(scene);

    if (!STORE.options.showPredictionMajor && !STORE.options.showPredictionMinor) {
      return;
    }

    if (_.isEmpty(world.object)) {
      return;
    }
    world.object.forEach((obj) => {
      const predictionLineColor = ObstacleColorMapping[obj.type] || DEFAULT_COLOR;
      const predictions = obj.prediction;
      if (_.isEmpty(predictions)) {
        return;
      }

      if (!STORE.options[`showObstacles${_.upperFirst(_.camelCase(obj.type))}`]) {
        return;
      }

      // Take the prediction line with highest probability as major, others as minor.
      _.sortBy(predictions, (o) => o.probability);
      const predictionMajor = predictions[predictions.length - 1];
      const predictionMinor = predictions.slice(0, predictions.length - 1);

      if (STORE.options.showPredictionMajor) {
        const predictedTraj = coordinates.applyOffsetToArray(
          predictionMajor.predictedTrajectory,
        );
        const mesh = drawSegmentsFromPoints(predictedTraj,
          predictionLineColor, majorThickness, 6);
        this.predLines.push(mesh);
        scene.add(mesh);

        // Draw circles and gaussian
        for (let j = 0; j < predictedTraj.length; j += 1) {
          const circleMesh = this.getPredCircle();
          circleMesh.position.set(predictedTraj[j].x, predictedTraj[j].y, 0.24);
          circleMesh.material.color.setHex(predictionLineColor);
          scene.add(circleMesh);

          this.drawGaussian(
            predictionMajor.predictedTrajectory[j].gaussianInfo,
            predictionLineColor,
            predictedTraj[j],
            scene,
          );
        }
      }

      let minorThickness = 2.3;
      if (STORE.options.showPredictionMinor) {
        predictionMinor.forEach((prediction) => {
          const traj = prediction.predictedTrajectory;
          const positions = coordinates.applyOffsetToArray(traj);
          const mesh = drawSegmentsFromPoints(
            positions, predictionLineColor, minorThickness, 6,
          );
          this.predLines.push(mesh);
          scene.add(mesh);

          for (let j = 0; j < traj.length; j += 1) {
            this.drawGaussian(
              traj[j].gaussianInfo, predictionLineColor, positions[j], scene,
            );
          }

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
      opacity: 0.5,
    });
    const circleMesh = drawCircle(0.2, material);
    this.predCircles.push(circleMesh);
    return circleMesh;
  }

  drawGaussian(gaussian, color, position, scene) {
    if (!STORE.options.showGaussianInfo) {
      return;
    }

    if (gaussian && gaussian.ellipseA > EPSILON && gaussian.ellipseB > EPSILON) {
      const material = new THREE.MeshBasicMaterial({
        color, transparent: true, opacity: 0.35,
      });
      const ellipseMesh = drawEllipse(
        gaussian.ellipseA, gaussian.ellipseB, material,
      );

      ellipseMesh.position.set(position.x, position.y, 0.25);
      ellipseMesh.rotation.set(0, 0, gaussian.thetaA);
      this.predGaussian.push(ellipseMesh);
      scene.add(ellipseMesh);
    }
  }
}
