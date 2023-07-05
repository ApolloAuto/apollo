import { observable, action } from 'mobx';
import * as THREE from 'three';
import { ObstacleTypeColorMap } from 'utils/constant';

export default class CameraData {
    @observable initPosition = observable.map();

    @observable deltaPosition = observable.map();

    @observable initStaticRotation = observable.map();

    @observable deltaStaticRotation = observable.map();

    @observable initDynamicRotation = observable.map();

    @observable deltaDynamicRotation = observable.map();

    @observable cameraChannels = [];

    @observable imageSrcData = null;

    @observable imageAspectRatio = null;

    constructor() {
      ['x', 'y', 'z'].forEach((axis) => {
        this.initPosition.set(axis, 0);
        this.deltaPosition.set(axis, 0);
        this.initStaticRotation.set(axis, 0);
        this.deltaStaticRotation.set(axis, 0);
        this.initDynamicRotation.set(axis, 0);
        this.deltaDynamicRotation.set(axis, 0);
      });
    }

    getPositionAndRotationFromMatrix(matrix) {
      const position = new THREE.Vector3();
      const quaternion = new THREE.Quaternion();
      const scale = new THREE.Vector3();
      matrix.decompose(position, quaternion, scale);
      const euler = new THREE.Euler().setFromQuaternion(quaternion);
      return { position, rotation: euler };
    }

    // The init camera data is being updated per frame
    @action init(data, coordinates) {
      console.log('camera data');
      console.log(data);
      // Camera image
      if (data.image && data.image.length > 0) {
        // 图片缩放比例
        const kImageScale = data.kImageScale;
        const imageData = new Uint8Array(data.image);
        const blob = new Blob([imageData], { type: 'image/jpeg' });
        const img = new Image();
        img.src = URL.createObjectURL(blob);
        // If there are obstacles
        if (data.bbox2d && data.bbox2d.length > 0) {
          img.onload = () => {
            const canvas = document.createElement('canvas');
            canvas.width = img.width;
            canvas.height = img.height;
            const ctx = canvas.getContext('2d');
            ctx.drawImage(img, 0, 0);
            data.bbox2d.forEach((bbox, index) => {
              const obstaclesId = data.obstaclesId[index] || '';
              const obstaclesSubType = data.obstaclesSubType[index] || '';

              ctx.strokeStyle = ObstacleTypeColorMap.get(obstaclesSubType) || 'red';

              let {xmin, ymin, xmax, ymax} = bbox;
              if (xmin === ymin === xmax === ymax) {
                return;
              }
              [xmin, ymin, xmax, ymax] = [xmin, ymin, xmax, ymax]
                .map((value) => value * kImageScale);
              ctx.strokeRect(xmin, ymin, xmax - xmin, ymax - ymin);
              ctx.fillStyle = ObstacleTypeColorMap.get(obstaclesSubType) || 'white';
              ctx.font = '16px Arial';
              ctx.fillText(`${obstaclesSubType.substring(3)}:${obstaclesId}`, xmin, ymin);
            });
            this.imageSrcData = canvas.toDataURL();
          };
        } else {
          this.imageSrcData = URL.createObjectURL(blob);
        }
      }

      this.imageAspectRatio = data.imageAspectRatio;

      // Camera dynamic transformation matrix: the dynamic localization matrix
      const localizationMatrix = new THREE.Matrix4();
      if (data.localization && coordinates.isInitialized()) {
        // The translation parameters x/y/z is located at 12/13/14 respectively
        const translation = coordinates.applyOffset(
          { x: data.localization[12], y: data.localization[13] });
        data.localization[12] = translation.x;
        data.localization[13] = translation.y;
        data.localization[14] = 0;
        localizationMatrix.fromArray(data.localization);
      }

      // Camera static transformation matrix: the matrix from localization to camera
      const localizationToCameraMatrix = new THREE.Matrix4();
      if (data.localization2cameraTf) {
        localizationToCameraMatrix.fromArray(data.localization2cameraTf);
      }

      const cameraMatrixInWorld = new THREE.Matrix4();
      cameraMatrixInWorld.multiplyMatrices(
        localizationMatrix, localizationToCameraMatrix,
      );

      const initStaticTransform = this.getPositionAndRotationFromMatrix(
        localizationToCameraMatrix,
      );
      const initDynamicTransform = this.getPositionAndRotationFromMatrix(localizationMatrix);
      const initOverallTransform = this.getPositionAndRotationFromMatrix(cameraMatrixInWorld);
      const geolocation = coordinates.applyOffset(initOverallTransform.position, true);

      // For rotation, we need static and dynamic one respectively;
      // for position, we only care about the overall position
      ['x', 'y', 'z'].forEach((axis) => {
        this.initPosition.set(axis, geolocation[axis]);
        this.initStaticRotation.set(axis, initStaticTransform.rotation[axis]);
        this.initDynamicRotation.set(axis, initDynamicTransform.rotation[axis]);
      });
    }

    // The delta camera data can be updated by manual input
    @action update(type, key, value) {
      const deltaMap = this[`delta${type}`];
      if (deltaMap && deltaMap.has(key)) {
        deltaMap.set(key, deltaMap.get(key) + value);
      }
    }

    get() {
      const position = {};
      const staticRotation = {};
      const dynamicRotation = {};
      ['x', 'y', 'z'].forEach((axis) => {
        position[axis] = this.initPosition.get(axis) + this.deltaPosition.get(axis);
        staticRotation[axis] = this.initStaticRotation.get(axis)
                + this.deltaStaticRotation.get(axis);
        dynamicRotation[axis] = this.initDynamicRotation.get(axis)
                + this.deltaDynamicRotation.get(axis);
      });

      // Combine static and dynamic rotation to calculate overall rotation
      const staticQuaternion = new THREE.Quaternion();
      const staticEuler = new THREE.Euler(
        staticRotation.x, staticRotation.y, staticRotation.z,
      );
      staticQuaternion.setFromEuler(staticEuler);

      const dynamicQuaternion = new THREE.Quaternion();
      const dynamicEuler = new THREE.Euler(
        dynamicRotation.x, dynamicRotation.y, dynamicRotation.z,
      );
      dynamicQuaternion.setFromEuler(dynamicEuler);

      const overallQuaternion = new THREE.Quaternion();
      overallQuaternion.multiplyQuaternions(dynamicQuaternion, staticQuaternion);
      const rotation = new THREE.Euler().setFromQuaternion(overallQuaternion);

      return { position, rotation };
    }

    @action reset() {
      ['x', 'y', 'z'].forEach((axis) => {
        this.deltaPosition.set(axis, 0);
        this.deltaStaticRotation.set(axis, 0);
        this.deltaDynamicRotation.set(axis, 0);
      });
    }
}
