import * as THREE from 'three';
import _ from 'lodash';

import { loadMaterial } from 'utils/models';

import trafficLightMaterial from 'assets/models/traffic_light.mtl';
import trafficLightObject from 'assets/models/traffic_light.obj';
import TrafficControlsBase from 'renderer/traffic_controls/traffic_controls_base';

const TRAFFIC_LIGHT_SCALE = 2.3;

const SUBSIGNAL_TO_INDEX = {
  GREEN: 6,
  YELLOW: 5,
  RED: 4,
};
const SUBSIGNAL_TO_LIGHT = {
  GREEN: 0x17f470,
  YELLOW: 0xcec832,
  RED: 0xff0000,
};

const EPSILON = 1e-9;

export default class TrafficSignals extends TrafficControlsBase {
  // Use the orthogonal direction of the traffic light boundary plane to get the
  // two potential headings, facing towards or away from the stop line. Then use
  // the stop line to get the real facing as the traffic light should always be
  // facing to rather than away from the stop line.
  static getHeadingFromStopLineAndTrafficLightBoundary(signal) {
    // find the plane of the signal
    const boundaryPoints = signal.boundary.point;
    if (boundaryPoints.length < 3) {
      console.warn(`Cannot get three points from boundary, signal_id: ${signal.id.id}`);
      return super.getHeadingFromStopLine(signal);
    }

    // get an orthogonal line of the plane (only need its projection on XY coordinate system)
    // construct ax+by+c=0 ==> orthogonalX*x+orthogonalY*y+constant=0
    const boundary1 = boundaryPoints[0];
    const boundary2 = boundaryPoints[1];
    const boundary3 = boundaryPoints[2];
    const orthogonalX = (boundary2.x - boundary1.x) * (boundary3.z - boundary1.z)
            - (boundary3.x - boundary1.x) * (boundary2.z - boundary1.z);
    const orthogonalY = (boundary2.y - boundary1.y) * (boundary3.z - boundary1.z)
            - (boundary3.y - boundary1.y) * (boundary2.z - boundary1.z);
    const orthogonalConstant = -orthogonalX * boundary1.x - orthogonalY * boundary1.y;

    // get the stop line
    const stopLine = _.get(signal, 'stopLine[0].segment[0].lineSegment.point', '');
    const len = stopLine.length;
    if (len < 2) {
      console.warn(`Cannot get any stop line, signal_id: ${signal.id.id}`);
      return NaN;
    }

    // construct ax+by+c=0 ==> stopLineX*x+stopLineY*y+constant=0
    const stopLineX = stopLine[len - 1].y - stopLine[0].y;
    const stopLineY = stopLine[0].x - stopLine[len - 1].x;
    const stopLineConstant = -stopLineX * stopLine[0].x - stopLineY * stopLine[0].y;

    // calculate the intersection
    if (Math.abs(stopLineX * orthogonalY - orthogonalX * stopLineY) < EPSILON) {
      console.warn('The signal orthogonal direction is parallel to the stop line,',
        `signal_id: ${signal.id.id}`);
      return super.getHeadingFromStopLine(signal);
    }
    const intersectX = (stopLineY * orthogonalConstant - orthogonalY * stopLineConstant)
            / (stopLineX * orthogonalY - orthogonalX * stopLineY);
    const intersectY = stopLineY !== 0
      ? (-stopLineX * intersectX - stopLineConstant) / stopLineY
      : (-orthogonalX * intersectX - orthogonalConstant) / orthogonalY;
    let direction = Math.atan2(-orthogonalX, orthogonalY);

    // if the direction is not towards to intersection point, turn around
    if ((direction < 0 && intersectY > boundary1.y)
            || (direction > 0 && intersectY < boundary1.y)) {
      direction += Math.PI;
    }
    return direction;
  }

  constructor() {
    super(trafficLightMaterial, trafficLightObject, TRAFFIC_LIGHT_SCALE);

    this.hash = -1;
    this.shiningSubsignals = [];

    // Prepare materials for red/yellow/green light
    loadMaterial(trafficLightMaterial, (materials) => {
      const [, , DARK_GREEN, DARK_YELLOW, DARK_RED] = materials.getAsArray();

      const getLightMaterial = (origMaterial, subsignal) => {
        const lightMaterial = origMaterial.clone();
        lightMaterial.emissive.set(SUBSIGNAL_TO_LIGHT[subsignal]);
        lightMaterial.dispose();
        return lightMaterial;
      };

      this.SUBSIGNAL_MATERIALS = {
        GREEN: {
          DARK: DARK_GREEN,
          LIGHT: getLightMaterial(DARK_GREEN, 'GREEN'),
        },
        YELLOW: {
          DARK: DARK_YELLOW,
          LIGHT: getLightMaterial(DARK_YELLOW, 'YELLOW'),
        },
        RED: {
          DARK: DARK_RED,
          LIGHT: getLightMaterial(DARK_RED, 'RED'),
        },
      };
    });
  }

  getPositionAndHeading(signal, coordinates) {
    const locations = [];
    signal.subsignal.forEach((subsignal) => {
      if (subsignal.location) {
        locations.push(subsignal.location);
      }
    });
    if (locations.length === 0) {
      console.warn('Subsignal locations not found, use signal boundary instead.');
      locations.push(signal.boundary.point);
    }
    if (locations.length === 0) {
      console.warn('Unable to determine signal location, skip.');
      return null;
    }
    const heading = TrafficSignals.getHeadingFromStopLineAndTrafficLightBoundary(signal);
    if (!isNaN(heading)) {
      let position = new THREE.Vector3(0, 0, 0);
      position.x = _.meanBy(_.values(locations), (l) => l.x);
      position.y = _.meanBy(_.values(locations), (l) => l.y);
      position = coordinates.applyOffset(position);
      return { pos: position, heading };
    }
    console.error('Error loading traffic light. Unable to determine heading.');
    return null;
  }

  clearTrafficLightStatus() {
    if (this.shiningSubsignals.length > 0) {
      this.shiningSubsignals.forEach((mesh) => {
        const darkMaterial = _.get(this.SUBSIGNAL_MATERIALS, `${mesh.subsignal}.DARK`);
        if (darkMaterial) {
          mesh.material = darkMaterial;
        }
      });
      this.shiningSubsignals = [];
    }
  }

  updateTrafficLightStatus(signals) {
    if (!Array.isArray(signals)) {
      return;
    }

    this.clearTrafficLightStatus();

    if (signals && this.object) {
      const signalId2Color = {};
      signals.forEach((signal) => signalId2Color[signal.id] = signal.currentSignal);

      Object.keys(this.object)
        .filter((id) => id in signalId2Color)
        .forEach((id) => {
          const mesh = this.object[id];
          if (mesh) {
            const subsignal = signalId2Color[id];
            const index = SUBSIGNAL_TO_INDEX[subsignal];
            if (index) {
              const lightMaterial = _.get(
                this.SUBSIGNAL_MATERIALS, `${subsignal}.LIGHT`,
              );

              const subsignalMesh = mesh.children[index];
              subsignalMesh.material = lightMaterial;
              subsignalMesh.subsignal = subsignal;

              this.shiningSubsignals.push(subsignalMesh);
            }
          }
        });
    }
  }
}
