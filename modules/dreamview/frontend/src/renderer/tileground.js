import * as THREE from "three";

import { loadTexture } from "utils/models";
import RENDERER from "renderer";
import { TRAFFIC_LIGHT, STOP_SIGN } from "renderer/traffic_control";

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
        this.type = "tile";

        this.hash = -1;
        this.currentTiles = {};
        this.currentItems = {};
        this.initialized = false;

        this.range = PARAMETERS.ground.defaults.tileRange;
        this.metadata = null;
        this.mapId = null;
        this.mapUrlPrefix = null;
    }

    initialize(serverUrl, metadata) {
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
        };

        this.mapId = metadata.mapid;
        this.mapUrlPrefix = `${this.metadata.imageUrl}/${this.mapId}`;
        this.totalSignals = metadata.signal;
        this.totalStopSigns = metadata.stopSign;
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

    // get signal and stopSign within current range
    getItems(row, col, key, totalItemsInMap, type) {
        const newItems = [];
        totalItemsInMap.forEach((item) => {
            if (isNaN(item.x) || isNaN(item.y) || isNaN(item.heading)) {
                return;
            }
            const { i, j } = this.getRowAndCol(item.x, item.y);
            if (i !== row || j !== col) {
                return;
            }
            item.type = type;
            newItems.push(item);

            if (!this.currentItems[type]) {
                this.currentItems[type] = [];
            }
            this.currentItems[type].push(`${key}_${item.id}`);
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

        loadTexture(mapUrl, texture => {
            texture.anisotropy = this.renderer.getMaxAnisotropy();
            const mesh = new THREE.Mesh(
                new THREE.PlaneGeometry(1, 1),
                new THREE.MeshBasicMaterial({map: texture}));

            mesh.position.set(position.x, position.y, position.z);
            mesh.scale.set(this.metadata.tileLength, this.metadata.tileLength, 1);
            mesh.overdraw = false;

            this.currentTiles[key] = mesh;
            scene.add(mesh);
        });
    }

    removeExpiredTiles(newTiles, scene) {
        for (const key in this.currentTiles){
            if (!newTiles.has(key)) {
                this.removeDrewObject(key, this.currentTiles, scene);
            }
        }

        Object.keys(this.currentItems).forEach((type) => {
            const newItemIds = [];
            this.currentItems[type].forEach((item) => {
                const [key, id] = item.split('_');
                if (newTiles.has(key)) {
                    newItemIds.push(id);
                }
            });
            RENDERER.removeTrafficControl(type, newItemIds);
        });
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
                    if (col < 0 || col > this.metadata.numCols ||
                        row < 0 || row > this.metadata.numRows) {
                        continue;
                    }
                    this.appendTiles(row, col, key, coordinates, scene);

                    if (this.totalSignals) {
                        RENDERER.addTrafficControl(
                            TRAFFIC_LIGHT,
                            this.getItems(row, col, key, this.totalSignals, TRAFFIC_LIGHT)
                        );
                    }
                    if (this.totalStopSigns) {
                        RENDERER.addTrafficControl(
                            STOP_SIGN,
                            this.getItems(row, col, key, this.totalStopSigns, STOP_SIGN)
                        );
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
        let newHash = "";
        for (let row = i-this.range; row <= i+this.range ; row++) {
            for (let col = j-this.range; col <= j+this.range ; col++) {
                const key = `${row},${col}`;
                newTiles.add(key);
                newHash += key;
            }
        }

        this.updateIndex(newHash, newTiles, coordinates, scene);
    }
}
