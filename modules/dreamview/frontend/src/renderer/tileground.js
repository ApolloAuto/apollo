import * as THREE from 'three';

import { loadTexture } from 'utils/models';
import TrafficSigns from 'renderer/traffic_controls/traffic_signs';
import TrafficSignals from 'renderer/traffic_controls/traffic_signals';

import stopSignMaterial from 'assets/models/stop_sign.mtl';
import stopSignObject from 'assets/models/stop_sign.obj';
import yieldSignMaterial from 'assets/models/yield_sign.mtl';
import yieldSignObject from 'assets/models/yield_sign.obj';

const STOP_SIGN_SCALE = 0.01;
const YIELD_SIGN_SCALE = 1.5;

function diffTiles(newTiles, currentTiles) {
  const difference = new Set(newTiles);
  for (const key in currentTiles) {
    difference.delete(key);
  }

  return difference;
}

export default class TileGround {
  constructor(renderer) {
    this.renderer = renderer;

    // This a dummy variable. Without it,
    // renderer will not proceed the render process.
    this.mesh = true;
    this.type = 'tile';

    this.range = PARAMETERS.ground.defaults.tileRange;
    this.metadata = null;
    this.mapId = null;
    this.mapUrlPrefix = null;

    this.hash = -1;
    this.currentTiles = {};
    this.currentTrafficLights = [];
    this.currentStopSigns = [];
    this.currentYieldSigns = [];
    this.initialized = false;

    this.trafficSignals = new TrafficSignals();
    this.stopSigns = new TrafficSigns(
      stopSignMaterial, stopSignObject, STOP_SIGN_SCALE,
    );
    this.yieldSigns = new TrafficSigns(
      yieldSignMaterial, yieldSignObject, YIELD_SIGN_SCALE,
    );
  }

  initialize(metadata) {
    this.metadata = {
      tileLength: metadata.tile * metadata.mpp,
      left: metadata.left,
      top: metadata.top,
      numCols: metadata.wnum,
      numRows: metadata.hnum,
      mpp: metadata.mpp,
      tile: metadata.tile,
      imageUrl: metadata.image_url,
      availableMapTiles: new Set(metadata.availableImages),

      allSignals: metadata.signal,
      allStopSigns: metadata.stopSign,
      allYieldSigns: metadata.yield,
    };

    this.mapId = metadata.mapid;
    this.mapUrlPrefix = `${this.metadata.imageUrl}/${this.mapId}`;
    this.initialized = true;
  }

  removeDrewObject(key, items, scene) {
    const object = items[key];
    if (object) {
      scene.remove(object);
      if (object.geometry) {
        object.geometry.dispose();
      }
      if (object.material) {
        object.material.dispose();
      }
    }
    delete items[key];
  }

  // get traffic-control items within current range
  getItems(row, col, totalItemsInMap) {
    const newItems = [];
    totalItemsInMap.forEach((item) => {
      if (isNaN(item.x) || isNaN(item.y) || isNaN(item.heading)) {
        return;
      }
      const { i, j } = this.getRowAndCol(item.x, item.y);
      if (i !== row || j !== col) {
        return;
      }
      newItems.push(item);
    });
    return newItems;
  }

  appendTiles(row, col, key, coordinates, scene) {
    const imageName = `${this.metadata.mpp}_${row}_${col}_${this.metadata.tile}.png`;
    if (!this.metadata.availableMapTiles.has(imageName)) {
      return;
    }

    const mapUrl = this.metadata.imageUrl
      ? `${this.mapUrlPrefix}/${imageName}`
      : `${this.mapUrlPrefix}?mapId=${this.mapId}&i=${row}&j=${col}`;

    const position = coordinates.applyOffset({
      x: this.metadata.left + (row + 0.5) * this.metadata.tileLength,
      y: this.metadata.top - (col + 0.5) * this.metadata.tileLength,
      z: 0,
    });

    loadTexture(mapUrl, (texture) => {
      texture.anisotropy = this.renderer.getMaxAnisotropy();
      const mesh = new THREE.Mesh(
        new THREE.PlaneGeometry(1, 1),
        new THREE.MeshBasicMaterial({ map: texture }),
      );

      mesh.position.set(position.x, position.y, position.z);
      mesh.scale.set(this.metadata.tileLength, this.metadata.tileLength, 1);
      mesh.overdraw = false;

      this.currentTiles[key] = mesh;
      scene.add(mesh);
    });
  }

  removeExpiredTiles(newTiles, scene) {
    for (const key in this.currentTiles) {
      if (!newTiles.has(key)) {
        this.removeDrewObject(key, this.currentTiles, scene);
      }
    }

    const currentTrafficLightIds = [];
    this.currentTrafficLights.forEach((item) => {
      const { key, id } = item;
      if (newTiles.has(key)) {
        currentTrafficLightIds.push(id);
      }
    });
    this.trafficSignals.removeExpired(currentTrafficLightIds, scene);

    const currentStopSignIds = [];
    this.currentStopSigns.forEach((item) => {
      const { key, id } = item;
      if (newTiles.has(key)) {
        currentStopSignIds.push(id);
      }
    });
    this.stopSigns.removeExpired(currentStopSignIds, scene);

    const currentYieldSignIds = [];
    this.currentYieldSigns.forEach((item) => {
      if (newTiles.has(item.key)) {
        currentYieldSignIds.push(item.id);
      }
    });
    this.yieldSigns.removeExpired(currentYieldSignIds, scene);
  }

  updateIndex(hash, newTiles, coordinates, scene) {
    if (hash !== this.hash) {
      this.hash = hash;
      this.removeExpiredTiles(newTiles, scene);

      const difference = diffTiles(newTiles, this.currentTiles);
      if (!_.isEmpty(difference) || !this.initialized) {
        for (const key of difference) {
          this.currentTiles[key] = null;
          const segments = key.split(',');
          const row = parseInt(segments[0]);
          const col = parseInt(segments[1]);
          if (col < 0 || col > this.metadata.numCols
                        || row < 0 || row > this.metadata.numRows) {
            continue;
          }
          this.appendTiles(row, col, key, coordinates, scene);

          if (this.metadata.allSignals) {
            const newItems = this.getItems(row, col, this.metadata.allSignals);
            this.trafficSignals.add(newItems, coordinates, scene);
            this.currentTrafficLights = [
              ...this.currentTrafficLights,
              ...newItems.map((item) => ({ key, id: item.id })),
            ];
          }

          if (this.metadata.allStopSigns) {
            const newItems = this.getItems(row, col, this.metadata.allStopSigns);
            this.stopSigns.add(newItems, coordinates, scene);
            this.currentStopSigns = [
              ...this.currentStopSigns,
              ...newItems.map((item) => ({ key, id: item.id })),
            ];
          }

          if (this.metadata.allYieldSigns) {
            const newItems = this.getItems(row, col, this.metadata.allYieldSigns);
            this.yieldSigns.add(newItems, coordinates, scene);
            this.currentYieldSigns = [
              ...this.currentYieldSigns,
              ...newItems.map((item) => ({ key, id: item.id })),
            ];
          }
        }
      }
    }
  }

  getRowAndCol(x, y) {
    const i = Math.floor((x - this.metadata.left) / this.metadata.tileLength);
    const j = Math.floor((this.metadata.top - y) / this.metadata.tileLength);
    return { i, j };
  }

  update(world, coordinates, scene) {
    if (!coordinates.isInitialized() || !this.initialized) {
      return;
    }

    const x = world.autoDrivingCar.positionX;
    const y = world.autoDrivingCar.positionY;

    const { i, j } = this.getRowAndCol(x, y);
    const newTiles = new Set();
    let newHash = '';
    for (let row = i - this.range; row <= i + this.range; row++) {
      for (let col = j - this.range; col <= j + this.range; col++) {
        const key = `${row},${col}`;
        newTiles.add(key);
        newHash += key;
      }
    }

    this.updateIndex(newHash, newTiles, coordinates, scene);
    this.trafficSignals.updateTrafficLightStatus(world.perceivedSignal);
  }
}
