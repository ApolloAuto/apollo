import { lowerFirst } from 'lodash';
import * as THREE from 'three';
import { LocalStorage, KEY_MANAGER } from '@dreamview/dreamview-core/src/util/storageManager';
import { cameraParams } from '../constant/common';

export default class View {
    private defaultViewDistance;

    private nearViewDistance;

    private overheadViewDistance;

    private mapViewDistance;

    private camera: THREE.PerspectiveCamera;

    private viewAngle;

    private controls;

    public viewType;

    private adc;

    private viewLocalStorage = new LocalStorage(KEY_MANAGER.CurrentSwitchView);

    public constructor(camera, controls, adc) {
        this.defaultViewDistance = 22.5;
        this.nearViewDistance = 11.25;
        this.overheadViewDistance = 90;
        this.mapViewDistance = 50;
        this.camera = camera;
        this.viewAngle = 0.8;
        this.controls = controls;
        this.adc = adc;
        this.viewType = this.viewLocalStorage.get('Default');
    }

    public setDefaultViewDistance(distance) {
        this.defaultViewDistance = distance;
    }

    public setNearViewDistance(distance) {
        this.nearViewDistance = distance;
    }

    public setOverheadViewDistance(distance) {
        this.overheadViewDistance = distance;
    }

    public setMapViewDistance(distance) {
        this.mapViewDistance = distance;
    }

    public setViewAngle(angle) {
        this.viewAngle = angle;
    }

    public setViewType(type, force = true) {
        this.viewType = type;
        if (force) {
            this.viewLocalStorage.set(type);
        }
    }

    public setView() {
        if (!this.adc) {
            return;
        }
        const target = this.adc?.adc;
        this.camera.fov = cameraParams[this.viewType].fov;
        this.camera.near = cameraParams[this.viewType].near;
        this.camera.far = cameraParams[this.viewType].far;
        const { x = 0, y = 0, z = 0 } = target?.position || {};
        const rotationY = target?.rotation.y || 0;
        const offsetX =
            this[`${lowerFirst(this.viewType)}ViewDistance`] * Math.cos(rotationY) * Math.cos(this.viewAngle);
        const offsetY =
            this[`${lowerFirst(this.viewType)}ViewDistance`] * Math.sin(rotationY) * Math.cos(this.viewAngle);
        const offsetZ = this[`${lowerFirst(this.viewType)}ViewDistance`] * Math.sin(this.viewAngle);
        switch (this.viewType) {
            case 'Default':
            case 'Near':
                this.camera.position.set(x - offsetX, y - offsetY, z + offsetZ);
                this.camera.up.set(0, 0, 1);
                this.camera.lookAt(x + offsetX, y + offsetY, 0);
                this.controls.enabled = false;
                break;
            case 'Overhead':
                this.camera.position.set(x, y, z + offsetZ);
                this.camera.up.set(0, 1, 0);
                this.camera.lookAt(x, y + offsetY / 8, z);
                this.controls.enabled = false;
                break;
            case 'Map':
                if (!this.controls.enabled) {
                    this.camera.position.set(x, y, z + this.mapViewDistance);
                    this.camera.up.set(0, 0, 1);
                    this.camera.lookAt(x, y, 0);
                    this.controls.enabled = true;
                    this.controls.enabledRotate = true;
                    this.controls.zoom0 = this.camera.zoom;
                    this.controls.target0 = new THREE.Vector3(x, y, 0);
                    this.controls.position0 = this.camera.position.clone();
                    this.controls.reset();
                }
                break;
            default:
                break;
        }
        this.camera.updateProjectionMatrix();
    }

    public updateViewDistance(offset) {
        if (this.viewType === 'Map') {
            this.controls.enabled = false;
        }
        const near = cameraParams[this.viewType].near;
        const far = cameraParams[this.viewType].far;
        const curDistance = this[`${lowerFirst(this.viewType)}ViewDistance`];
        let newDistance = Math.min(far, curDistance + offset);
        newDistance = Math.max(near, curDistance + offset);
        this[`set${this.viewType}ViewDistance`](newDistance);
        this.setView();
    }

    public changeViewType(type, force = true) {
        this.setViewType(type, force);
        this.setView();
    }
}
