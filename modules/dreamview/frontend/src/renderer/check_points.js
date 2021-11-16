import { drawSegmentsFromPoints, disposeMesh } from 'utils/draw';

const DELTA_Z_OFFSET = 0.1;

const COLOR_MAPPING = {
  MIDWAY: 0xFF7F50,
  END: 0xFFDAB9,
};

export default class CheckPoints {
  constructor(coordinates, scene) {
    this.coordinates = coordinates;
    this.scene = scene;

    this.mesh = [];
  }

  dispose() {
    this.mesh.forEach((point) => {
      disposeMesh(point);
      this.scene.remove(point);
    });

    this.mesh = [];
  }

  update(checkpoints) {
    if (!this.coordinates.isInitialized()) {
      return;
    }

    if (!checkpoints || !Array.isArray(checkpoints)) {
      return;
    }

    this.dispose();

    checkpoints.forEach((checkpoint) => {
      const color = checkpoint.isMidway ? COLOR_MAPPING.MIDWAY : COLOR_MAPPING.END;
      const points = checkpoint.region.map(
        (point) => this.coordinates.applyOffset({ ...point, z: DELTA_Z_OFFSET }));
      points.push(points[0]);

      const region = drawSegmentsFromPoints(points, color, 3);
      this.scene.add(region);

      this.mesh.push(region);
    });
  }
}
