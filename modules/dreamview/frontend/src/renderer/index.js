import * as THREE from 'three';
import Stats from 'stats.js';

import Styles from 'styles/main.scss';

import Coordinates from 'renderer/coordinates';
import AutoDrivingCar from 'renderer/adc';
import CheckPoints from 'renderer/check_points.js';
import Ground from 'renderer/ground';
import TileGround from 'renderer/tileground';
import Map from 'renderer/map';
import PlanningTrajectory from 'renderer/trajectory.js';
import PlanningStatus from 'renderer/status.js';
import PerceptionObstacles from 'renderer/obstacles.js';
import Decision from 'renderer/decision.js';
import Prediction from 'renderer/prediction.js';
import Routing from 'renderer/routing.js';
import RoutingEditor from 'renderer/routing_editor.js';
import Gnss from 'renderer/gnss.js';
import PointCloud from 'renderer/point_cloud.js';

const _ = require('lodash');

class Renderer {
  constructor() {
    // Disable antialias for mobile devices.
    const useAntialias = !this.isMobileDevice();

    this.coordinates = new Coordinates();
    this.renderer = new THREE.WebGLRenderer({
      antialias: useAntialias,
      // Transparent background
      alpha: true,
    });
    this.scene = new THREE.Scene();
    if (OFFLINE_PLAYBACK) {
      this.scene.background = new THREE.Color(0x000C17);
    }

    // The dimension of the scene
    this.dimension = {
      width: 0,
      height: 0,
    };

    // The ground.
    this.ground = (PARAMETERS.ground.type === 'tile' || OFFLINE_PLAYBACK)
      ? new TileGround(this.renderer) : new Ground();

    // The map.
    this.map = new Map();

    // The main autonomous driving car.
    this.adc = new AutoDrivingCar('adc', this.scene);

    // The car that projects the starting point of the planning trajectory
    this.planningAdc = OFFLINE_PLAYBACK ? null : new AutoDrivingCar('planningAdc', this.scene);

    // The shadow localization
    this.shadowAdc = new AutoDrivingCar('shadowAdc', this.scene);

    // The planning trajectory.
    this.planningTrajectory = new PlanningTrajectory();

    // The planning status
    this.planningStatus = new PlanningStatus();

    // The perception obstacles.
    this.perceptionObstacles = new PerceptionObstacles();

    // The decision.
    this.decision = new Decision();

    // The prediction.
    this.prediction = new Prediction();

    // The routing.
    this.routing = new Routing();

    // The route editor
    this.routingEditor = new RoutingEditor();
    this.routingPoint = null;

    // Distinguish between drawing point and drawing arrow
    this.startMove = false;

    // The GNSS/GPS
    this.gnss = new Gnss();

    this.pointCloud = new PointCloud();

    this.checkPoints = OFFLINE_PLAYBACK && new CheckPoints(this.coordinates, this.scene);

    // The Performance Monitor
    this.stats = null;
    if (PARAMETERS.debug.performanceMonitor) {
      this.stats = new Stats();
      this.stats.showPanel(1);
      this.stats.domElement.style.position = 'absolute';
      this.stats.domElement.style.top = null;
      this.stats.domElement.style.bottom = '0px';
      document.body.appendChild(this.stats.domElement);
    }

    // Geolocation of the mouse
    this.geolocation = { x: 0, y: 0 };
  }

  initialize(canvasId, width, height, options, cameraData) {
    this.options = options;
    this.cameraData = cameraData;
    this.canvasId = canvasId;

    // Camera
    this.viewAngle = PARAMETERS.camera.viewAngle;
    this.viewDistance = (
      PARAMETERS.camera.laneWidth
            * PARAMETERS.camera.laneWidthToViewDistanceRatio);
    this.camera = new THREE.PerspectiveCamera(
      PARAMETERS.camera[this.options.cameraAngle].fov,
      width / height,
      PARAMETERS.camera[this.options.cameraAngle].near,
      PARAMETERS.camera[this.options.cameraAngle].far,
    );
    this.camera.name = 'camera';
    this.scene.add(this.camera);

    this.updateDimension(width, height);
    this.renderer.setPixelRatio(window.devicePixelRatio);

    const container = document.getElementById(canvasId);
    container.appendChild(this.renderer.domElement);

    const ambient = new THREE.AmbientLight(0x444444);
    const directionalLight = new THREE.DirectionalLight(0xffeedd);
    directionalLight.position.set(0, 0, 1).normalize();

    // The orbit axis of the OrbitControl depends on camera's up vector
    // and can only be set during creation of the controls. Thus,
    // setting camera up here. Note: it's okay if the camera.up doesn't
    // match the point of view setting, the value will be adjusted during
    // each update cycle.
    this.camera.up.set(0, 0, 1);

    // Orbit control for moving map
    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enable = false;

    // handler for route editing with mouse down events
    this.onMouseDownHandler = this.editRoute.bind(this);
    this.onMouseMoveHandler = this.onMouseMoveHandler.bind(this);
    this.onMouseUpHandler = this.onMouseUpHandler.bind(this);

    this.scene.add(ambient);
    this.scene.add(directionalLight);

    // TODO maybe add sanity check.

    // Actually start the animation.
    this.animate();
  }

  maybeInitializeOffest(x, y, forced_update = false) {
    if (!this.coordinates.isInitialized() || forced_update) {
      this.coordinates.initialize(x, y);
    }
  }

  updateDimension(width, height) {
    if (width < Styles.MIN_MAIN_VIEW_WIDTH / 2 && this.dimension.width >= width) {
      // Reach minimum, do not update camera/renderer dimension anymore.
      return;
    }

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);

    this.dimension.width = width;
    this.dimension.height = height;
  }

  enableOrbitControls(enableRotate) {
    // update camera
    const carPosition = this.adc.mesh.position;
    this.camera.position.set(carPosition.x, carPosition.y, 50);
    if (this.coordinates.systemName === 'FLU') {
      this.camera.up.set(1, 0, 0);
    } else {
      this.camera.up.set(0, 0, 1);
    }
    const lookAtPosition = new THREE.Vector3(carPosition.x, carPosition.y, 0);
    this.camera.lookAt(lookAtPosition);

    // update control reset values to match current camera's
    this.controls.target0 = lookAtPosition.clone();
    this.controls.position0 = this.camera.position.clone();
    this.controls.zoom0 = this.camera.zoom;

    // set distance control
    this.controls.minDistance = 4;
    this.controls.maxDistance = 4000;

    // set vertical angle control
    this.controls.minPolarAngle = 0;
    this.controls.maxPolarAngle = Math.PI / 2;

    this.controls.enabled = true;
    this.controls.enableRotate = enableRotate;
    this.controls.reset();
  }

  adjustCamera(target, pov) {
    if (this.routingEditor.isInEditingMode()) {
      return;
    }

    this.camera.fov = PARAMETERS.camera[pov].fov;
    this.camera.near = PARAMETERS.camera[pov].near;
    this.camera.far = PARAMETERS.camera[pov].far;

    switch (pov) {
      case 'Default':
        let deltaX = (this.viewDistance * Math.cos(target.rotation.y)
                * Math.cos(this.viewAngle));
        let deltaY = (this.viewDistance * Math.sin(target.rotation.y)
                * Math.cos(this.viewAngle));
        let deltaZ = this.viewDistance * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x - deltaX;
        this.camera.position.y = target.position.y - deltaY;
        this.camera.position.z = target.position.z + deltaZ;
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt({
          x: target.position.x + deltaX,
          y: target.position.y + deltaY,
          z: 0,
        });

        this.controls.enabled = false;
        break;
      case 'Near':
        deltaX = (this.viewDistance * 0.5 * Math.cos(target.rotation.y)
                    * Math.cos(this.viewAngle));
        deltaY = (this.viewDistance * 0.5 * Math.sin(target.rotation.y)
                    * Math.cos(this.viewAngle));
        deltaZ = this.viewDistance * 0.5 * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x - deltaX;
        this.camera.position.y = target.position.y - deltaY;
        this.camera.position.z = target.position.z + deltaZ;
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt({
          x: target.position.x + deltaX,
          y: target.position.y + deltaY,
          z: 0,
        });

        this.controls.enabled = false;
        break;
      case 'Overhead':
        deltaY = (this.viewDistance * 0.5 * Math.sin(target.rotation.y)
                    * Math.cos(this.viewAngle));
        deltaZ = this.viewDistance * 2 * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x;
        this.camera.position.y = target.position.y + deltaY;
        this.camera.position.z = (target.position.z + deltaZ) * 2;
        if (this.coordinates.systemName === 'FLU') {
          this.camera.up.set(1, 0, 0);
        } else {
          this.camera.up.set(0, 1, 0);
        }
        this.camera.lookAt({
          x: target.position.x,
          y: target.position.y + deltaY,
          z: 0,
        });

        this.controls.enabled = false;
        break;
      case 'Map':
        if (!this.controls.enabled) {
          this.enableOrbitControls(true);
        }
        break;
      case 'CameraView': {
        const { position, rotation } = this.cameraData.get();

        const { x, y, z } = this.coordinates.applyOffset(position);
        this.camera.position.set(x, y, z);

        // Threejs camera is default facing towards to Z-axis negative direction,
        // but the actual camera is looking at Z-axis positive direction. So we need
        // to adjust the camera rotation considering the default camera orientation.
        this.camera.rotation.set(rotation.x + Math.PI, -rotation.y, -rotation.z);

        this.controls.enabled = false;

        const image = document.getElementById('camera-image');
        if (image && this.cameraData.imageSrcData) {
          image.src = this.cameraData.imageSrcData;
        }

        break;
      }
    }

    this.camera.updateProjectionMatrix();
  }

  enableRouteEditing() {
    this.enableOrbitControls(false);
    this.routingEditor.enableEditingMode(this.camera, this.adc);

    document.getElementById(this.canvasId).addEventListener('mousedown',
      this.onMouseDownHandler,
      false);
    document.getElementById(this.canvasId).addEventListener('mouseup',
      this.onMouseUpHandler,
      false);
    document.getElementById(this.canvasId).addEventListener('mousemove',
      this.onMouseMoveHandler,
      false);
  }

  disableRouteEditing() {
    this.routingEditor.disableEditingMode(this.scene);

    const element = document.getElementById(this.canvasId);
    if (element) {
      element.removeEventListener('mousedown',
        this.onMouseDownHandler,
        false);
      element.removeEventListener('mouseup',
        this.onMouseUpHandler,
        false);
      element.removeEventListener('mousemove',
        this.onMouseMoveHandler,
        false);
      this.startMove = false;
      this.routingPoint = null;
    }
  }

  addDefaultEndPoint(points) {
    for (let i = 0; i < points.length; i++) {
      this.routingEditor.addRoutingPoint(points[i], this.coordinates, this.scene, true);
    }
  }

  addDefaultRouting(routingName) {
    return this.routingEditor.addDefaultRouting(routingName, this.coordinates);
  }

  removeInvalidRoutingPoint(pointId, error) {
    const index = this.routingEditor.removeInvalidRoutingPoint(pointId, error, this.scene);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  setParkingInfo(info) {
    this.routingEditor.setParkingInfo(info);
  }

  removeAllRoutingPoints() {
    const indexArr = this.routingEditor.removeAllRoutePoints(this.scene);
    if (!_.isEmpty(indexArr)) {
      indexArr.forEach(item => {
        this.map.changeSelectedParkingSpaceColor(item, 0xDAA520);
      });
    }
  }

  removeLastRoutingPoint() {
    const index = this.routingEditor.removeLastRoutingPoint(this.scene);
    if (index !== -1) {
      this.map.changeSelectedParkingSpaceColor(index, 0xDAA520);
    }
  }

  sendRoutingRequest(points = []) {
    return this.routingEditor.sendRoutingRequest(this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates, points);
  }

  sendCycleRoutingRequest(defaultRoutingName, points, cycleNumber) {
    return this.routingEditor.sendCycleRoutingRequest(
      defaultRoutingName,
      points,
      cycleNumber,
      this.adc.mesh.position,
      this.adc.mesh.rotation.y,
      this.coordinates);
  }

  editRoute(event) {
    // Distinguish between operating on the screen and
    // selecting points on the screen
    if (event.target && !_.isEqual('CANVAS', event.target.tagName)) {
      return;
    }
    if (!this.routingEditor.isInEditingMode() || event.button !== THREE.MOUSE.LEFT) {
      return;
    }

    // return if the ground or coordinates is not loaded yet
    if (!this.coordinates.isInitialized() || !this.ground.mesh) {
      return;
    }

    this.routingPoint = this.getGeolocation(event);
  }

  onMouseMoveHandler(event) {
    if (this.routingPoint) {
      this.routingEditor.drawRoutingPointArrow(
        this.getGeolocation(event), this.routingPoint, this.coordinates, this.scene, this.startMove,
      );
      this.startMove = true;
    }
  }

  onMouseUpHandler() {
    if (this.routingPoint) {
      const selectedParkingSpaceIndex = this.routingEditor.addRoutingPoint(
        this.routingPoint, this.coordinates, this.scene, false,
      );
      if (selectedParkingSpaceIndex !== -1) {
        this.map.changeSelectedParkingSpaceColor(selectedParkingSpaceIndex);
      }
    }
    this.routingPoint = null;
    this.startMove = false;
  }

  // Render one frame. This supports the main draw/render loop.
  render() {
    // TODO should also return when no need to update.
    if (!this.coordinates.isInitialized()) {
      return;
    }

    // Return if the car mesh is not loaded yet, or the ground is not
    // loaded yet.
    if (!this.adc.mesh || !this.ground.mesh) {
      return;
    }

    // Upon the first time in render() it sees ground mesh loaded,
    // added it to the scene.
    if (this.ground.type === 'default' && !this.ground.initialized) {
      this.ground.initialize(this.coordinates);
      this.ground.mesh.name = 'ground';
      this.scene.add(this.ground.mesh);
    }

    if (this.pointCloud.initialized === false) {
      this.pointCloud.initialize();
      this.scene.add(this.pointCloud.points);
    }

    this.adjustCamera(this.adc.mesh, this.options.cameraAngle);
    this.renderer.render(this.scene, this.camera);
  }

  animate() {
    requestAnimationFrame(() => {
      this.animate();
    });

    if (this.stats) {
      this.stats.update();
    }
    this.render();
  }

  updateWorld(world) {
    const adcPose = world.autoDrivingCar;
    this.adc.update(this.coordinates, adcPose);
    if (!_.isNumber(adcPose.positionX) || !_.isNumber(adcPose.positionY)) {
      console.error(`Invalid ego car position: ${adcPose.positionX}, ${adcPose.positionY}!`);
      return;
    }

    this.adc.updateRssMarker(world.isRssSafe);
    this.ground.update(world, this.coordinates, this.scene);
    this.planningTrajectory.update(world, world.planningData, this.coordinates, this.scene);
    this.planningStatus.update(world.planningData, this.coordinates, this.scene);

    const isBirdView = ['Overhead', 'Map'].includes(_.get(this, 'options.cameraAngle'));
    this.perceptionObstacles.update(world, this.coordinates, this.scene, isBirdView);
    this.decision.update(world, this.coordinates, this.scene);
    this.prediction.update(world, this.coordinates, this.scene);
    this.updateRouting(world.routingTime, world.routePath);
    this.gnss.update(world, this.coordinates, this.scene);
    this.map.update(world);

    const planningAdcPose = _.get(world, 'planningData.initPoint.pathPoint');
    if (this.planningAdc && planningAdcPose) {
      const pose = {
        positionX: planningAdcPose.x,
        positionY: planningAdcPose.y,
        heading: planningAdcPose.theta,
      };
      this.planningAdc.update(this.coordinates, pose);
    }

    const shadowLocalizationPose = world.shadowLocalization;
    if (shadowLocalizationPose) {
      const shadowAdcPose = {
        positionX: shadowLocalizationPose.positionX,
        positionY: shadowLocalizationPose.positionY,
        heading: shadowLocalizationPose.heading,
      };
      this.shadowAdc.update(this.coordinates, shadowAdcPose);
    }
  }

  updateRouting(routingTime, routePath) {
    this.routing.update(routingTime, routePath, this.coordinates, this.scene);
  }

  updateGroundImage(mapName) {
    this.ground.updateImage(mapName);
  }

  updateGroundMetadata(mapInfo) {
    this.ground.initialize(mapInfo);
  }

  updateMap(newData, removeOldMap = false) {
    if (removeOldMap) {
      this.map.removeAllElements(this.scene);
    }
    this.map.appendMapData(newData, this.coordinates, this.scene);
    if (newData.parkingSpace) {
      this.routingEditor.setParkingSpaceInfo(
        newData.parkingSpace, this.coordinates
      );
    }
  }

  updatePointCloud(pointCloud) {
    if (!this.coordinates.isInitialized() || !this.adc.mesh) {
      return;
    }
    this.pointCloud.update(pointCloud, this.adc.mesh);
  }

  updateMapIndex(hash, elementIds, radius) {
    if (!this.routingEditor.isInEditingMode()
            || PARAMETERS.routingEditor.radiusOfMapRequest === radius) {
      this.map.updateIndex(hash, elementIds, this.scene);
    }
  }

  isMobileDevice() {
    return navigator.userAgent.match(/Android/i)
            || navigator.userAgent.match(/webOS/i)
            || navigator.userAgent.match(/iPhone/i)
            || navigator.userAgent.match(/iPad/i)
            || navigator.userAgent.match(/iPod/i);
  }

  getGeolocation(event) {
    if (!this.coordinates.isInitialized()) {
      return;
    }

    const canvasPosition = event.currentTarget.getBoundingClientRect();

    const vector = new THREE.Vector3(
      ((event.clientX - canvasPosition.left) / this.dimension.width) * 2 - 1,
      -((event.clientY - canvasPosition.top) / this.dimension.height) * 2 + 1,
      0,
    );

    vector.unproject(this.camera);

    const direction = vector.sub(this.camera.position).normalize();
    const distance = -this.camera.position.z / direction.z;
    const pos = this.camera.position.clone().add(direction.multiplyScalar(distance));
    const geo = this.coordinates.applyOffset(pos, true);

    return geo;
  }

  // Debugging purpose function:
  //  For detecting names of the lanes that your mouse cursor points to.
  getMouseOverLanes(event) {
    const canvasPosition = event.currentTarget.getBoundingClientRect();
    const mouse = new THREE.Vector3(
      ((event.clientX - canvasPosition.left) / this.dimension.width) * 2 - 1,
      -((event.clientY - canvasPosition.top) / this.dimension.height) * 2 + 1,
      0,
    );

    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, this.camera);
    const objects = this.map.data.lane.reduce(
      (result, current) => result.concat(current.drewObjects), []);
    const intersects = raycaster.intersectObjects(objects);
    const names = intersects.map((intersect) => intersect.object.name);
    return names;
  }

  checkCycleRoutingAvailable(points, threshold) {
    return this.routingEditor.checkCycleRoutingAvailable(points,
      this.adc.mesh.position, threshold);
  }
}

const RENDERER = new Renderer();

export default RENDERER;
