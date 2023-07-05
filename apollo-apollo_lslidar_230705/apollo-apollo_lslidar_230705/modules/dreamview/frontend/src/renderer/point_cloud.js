import * as THREE from 'three';

const MAX_POINTS = 200000;
const HEIGHT_COLOR_MAPPING = {
  0.5: 0xFF0000,
  1.0: 0xFF7F00,
  1.5: 0xFFFF00,
  2.0: 0x00FF00,
  2.5: 0x0000FF,
  3.0: 0x4B0082,
  10.0: 0x9400D3,
};

export default class PointCloud {
  constructor() {
    this.points = null;
    this.initialized = false;
  }

  initialize() {
    this.points = this.createPointCloud(HEIGHT_COLOR_MAPPING[0.5]);
    this.initialized = true;
  }

  createPointCloud(hex_color) {
    const geometry = new THREE.Geometry();
    const colors = [];
    for (let i = 0; i < MAX_POINTS; ++i) {
      const vertex = new THREE.Vector3();
      vertex.set(0, 0, -10);
      geometry.vertices.push(vertex);

      colors[i] = new THREE.Color(hex_color);
    }
    geometry.colors = colors;

    const material = new THREE.PointsMaterial({
      size: 0.25,
      transparent: true,
      opacity: 0.7,
      vertexColors: THREE.VertexColors,
    });
    const points = new THREE.Points(geometry, material);
    points.frustumCulled = false;
    return points;
  }

  update(pointCloud, adcMesh) {
    if (this.points === null) {
      return;
    }
    if (pointCloud.num.length % 3 !== 0) {
      console.warn('PointCloud length should be multiples of 3!');
      return;
    }
    const pointCloudSize = pointCloud.num.length / 3;
    const total = (pointCloudSize < MAX_POINTS) ? pointCloudSize : MAX_POINTS;
    let colorKey = 0.5;
    for (let i = 0; i < total; i++) {
      const x = pointCloud.num[i * 3];
      const y = pointCloud.num[i * 3 + 1];
      const z = pointCloud.num[i * 3 + 2];
      this.points.geometry.vertices[i].set(x, y, z + 0.8);
      // Update color based on height.
      if (z < 0.5) {
        colorKey = 0.5;
      } else if (z < 1.0) {
        colorKey = 1.0;
      } else if (z < 1.5) {
        colorKey = 1.5;
      } else if (z < 2.0) {
        colorKey = 2.0;
      } else if (z < 2.5) {
        colorKey = 2.5;
      } else if (z < 3.0) {
        colorKey = 3.0;
      } else {
        colorKey = 10.0;
      }
      this.points.geometry.colors[i].setHex(HEIGHT_COLOR_MAPPING[colorKey]);
    }
    // Hide unused points.
    for (let i = total; i < MAX_POINTS; ++i) {
      this.points.geometry.vertices[i].set(0, 0, -10);
    }
    this.points.geometry.verticesNeedUpdate = true;
    this.points.geometry.colorsNeedUpdate = true;
    this.points.position.set(adcMesh.position.x, adcMesh.position.y, adcMesh.position.z);
    this.points.rotation.set(0, 0, adcMesh.rotation.y);
  }
}
