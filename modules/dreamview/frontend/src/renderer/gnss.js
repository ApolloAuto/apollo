import * as THREE from "three";
import STORE from "store";
import { drawSegmentsFromPoints, drawCircle } from "utils/draw";

export default class GNSS {
  constructor() {
    this.circle = null;
    this.base = null;
  }

  update(world, coordinates, scene) {
    if (!world.gps || !world.autoDrivingCar) {
      return;
    }

    if (!this.circle) {
      const material = new THREE.MeshBasicMaterial({
        color: 0x006aff,
        transparent: false,
        opacity: 0.5
      });
      this.circle = drawCircle(0.2, material);
      scene.add(this.circle);
    }

    if (!this.base) {
      const config = STORE.hmi.vehicleParam;
      this.base = drawSegmentsFromPoints(
        [ new THREE.Vector3(config.frontEdgeToCenter, -config.leftEdgeToCenter, 0),
          new THREE.Vector3(config.frontEdgeToCenter, config.rightEdgeToCenter, 0),
          new THREE.Vector3(-config.backEdgeToCenter, config.rightEdgeToCenter, 0),
          new THREE.Vector3(-config.backEdgeToCenter, -config.leftEdgeToCenter, 0),
          new THREE.Vector3(config.frontEdgeToCenter, -config.leftEdgeToCenter, 0)],
        0x006aff,
        2,
        5
      );
      scene.add(this.base);
    }

    const visible = STORE.options["showPositionGps"];
    const position = coordinates.applyOffset({
      x: world.gps.positionX,
      y: world.gps.positionY,
      z: 0.01,
    });

    this.circle.position.set(position.x, position.y, position.z);
    this.circle.visible = visible;

    this.base.position.set(position.x, position.y, position.z);
    this.base.rotation.set(0, 0, world.gps.heading);
    this.base.visible = visible;
  }
}
