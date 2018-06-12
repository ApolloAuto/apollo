import * as THREE from "three";
import OrbitControls from "three/examples/js/controls/OrbitControls.js";
import Stats from "stats.js";

import PARAMETERS from "store/config/parameters.yml";
import Coordinates from "renderer/coordinates";
import AutoDrivingCar from "renderer/adc";
import Ground from "renderer/ground";
import TileGround from "renderer/tileground";
import Map from "renderer/map";
import PlanningTrajectory from "renderer/trajectory.js";
import PerceptionObstacles from "renderer/obstacles.js";
import Decision from "renderer/decision.js";
import Prediction from "renderer/prediction.js";
import Routing from "renderer/routing.js";
import RoutingEditor from "renderer/routing_editor.js";
import Gnss from "renderer/gnss.js";
import PointCloud from "renderer/point_cloud.js";

const _ = require('lodash');

class Renderer {
    constructor() {
        // Disable antialias for mobile devices.
        const useAntialias = !this.isMobileDevice();

        this.coordinates = new Coordinates();
        this.renderer = new THREE.WebGLRenderer({
            antialias: useAntialias
        });
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x000C17);

        // The dimension of the scene
        this.dimension = {
            width: 0,
            height: 0,
        };

        // The ground.
        this.ground = (PARAMETERS.ground.type === 'tile' || OFFLINE_PLAYBACK)
                      ? new TileGround() : new Ground();

        // The map.
        this.map = new Map();

        // The main autonomous driving car.
        this.adc = new AutoDrivingCar('adc', this.scene);

        // The car that projects the starting point of the planning trajectory
        this.planningAdc = OFFLINE_PLAYBACK ? null : new AutoDrivingCar('planningAdc', this.scene);

        // The planning trajectory.
        this.planningTrajectory = new PlanningTrajectory();

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

        // The GNSS/GPS
        this.gnss = new Gnss();

        this.pointCloud = new PointCloud();

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
        this.geolocation = {x: 0, y:0};
    }

    initialize(canvasId, width, height, options) {
        this.options = options;
        this.canvasId = canvasId;

        // Camera
        this.viewAngle = PARAMETERS.camera.viewAngle;
        this.viewDistance = (
            PARAMETERS.camera.laneWidth *
            PARAMETERS.camera.laneWidthToViewDistanceRatio);
        this.camera = new THREE.PerspectiveCamera(
            PARAMETERS.camera[this.options.cameraAngle].fov,
            window.innerWidth / window.innerHeight,
            PARAMETERS.camera[this.options.cameraAngle].near,
            PARAMETERS.camera[this.options.cameraAngle].far
        );
        this.camera.name = "camera";
        this.scene.add(this.camera);

        this.updateDimension(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);

        const container = document.getElementById(canvasId);
        container.appendChild(this.renderer.domElement);

        const ambient = new THREE.AmbientLight(0x444444);
        const directionalLight = new THREE.DirectionalLight(0xffeedd);
        directionalLight.position.set(0, 0, 1).normalize();

        // Orbit control for moving map
        this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enable = false;

        // handler for route editing with mouse down events
        this.onMouseDownHandler = this.editRoute.bind(this);

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
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);

        this.dimension.width = width;
        this.dimension.height = height;
    }

    enableOrbitControls() {
        const carPosition = this.adc.mesh.position;
        this.controls.enabled = true;
        this.controls.enableRotate = false;
        this.controls.reset();
        this.controls.minDistance = 20;
        this.controls.maxDistance = 1000;
        this.controls.target.set(carPosition.x, carPosition.y, 0);

        this.camera.position.set(carPosition.x, carPosition.y, 50);
        if (this.coordinates.systemName === "FLU") {
            this.camera.up.set(1, 0, 0);
        } else {
            this.camera.up.set(0, 1, 0);
        }
        this.camera.lookAt(carPosition.x, carPosition.y, 0);
    }

    adjustCamera(target, pov) {
        if (this.routingEditor.isInEditingMode()) {
            return;
        }

        this.camera.fov = PARAMETERS.camera[pov].fov;
        this.camera.near = PARAMETERS.camera[pov].near;
        this.camera.far = PARAMETERS.camera[pov].far;

        switch(pov) {
        case "Default":
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
                x: target.position.x + 2 * deltaX,
                y: target.position.y + 2 * deltaY,
                z: 0
            });

            this.controls.enabled = false;
            break;
        case "Near":
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
                x: target.position.x + 2 * deltaX,
                y: target.position.y + 2 * deltaY,
                z: 0
            });

            this.controls.enabled = false;
            break;
        case "Overhead":
            deltaY = (this.viewDistance * 0.5 * Math.sin(target.rotation.y)
                    * Math.cos(this.viewAngle));
            deltaZ = this.viewDistance * 2 * Math.sin(this.viewAngle);

            this.camera.position.x = target.position.x;
            this.camera.position.y = target.position.y + deltaY;
            this.camera.position.z = (target.position.z + deltaZ) * 2;
            if (this.coordinates.systemName === "FLU") {
                this.camera.up.set(1, 0, 0);
            } else {
                this.camera.up.set(0, 1, 0);
            }
            this.camera.lookAt({
                x: target.position.x,
                y: target.position.y + deltaY,
                z: 0
            });

            this.controls.enabled = false;
            break;
        case "Map":
            if (!this.controls.enabled) {
                this.enableOrbitControls();
            }
            break;
        }
        this.camera.updateProjectionMatrix();
    }

    enableRouteEditing() {
        this.enableOrbitControls();
        this.routingEditor.enableEditingMode(this.camera, this.adc);

        document.getElementById(this.canvasId).addEventListener('mousedown',
                                                                this.onMouseDownHandler,
                                                                false);
    }

    disableRouteEditing() {
        this.routingEditor.disableEditingMode(this.scene);

        document.getElementById(this.canvasId).removeEventListener('mousedown',
                                                                   this.onMouseDownHandler,
                                                                   false);
    }

    addDefaultEndPoint(points) {
        for (let i = 0; i < points.length; i++) {
            this.routingEditor.addRoutingPoint(points[i], this.coordinates, this.scene);
        }
    }

    removeAllRoutingPoints() {
        this.routingEditor.removeAllRoutePoints(this.scene);
    }

    removeLastRoutingPoint() {
        this.routingEditor.removeLastRoutingPoint(this.scene);
    }

    sendRoutingRequest() {
        return this.routingEditor.sendRoutingRequest(this.adc.mesh.position,
                                                     this.coordinates);
    }

    editRoute(event) {
        if (!this.routingEditor.isInEditingMode() || event.button !== THREE.MOUSE.LEFT) {
            return;
        }

        // return if the ground or coordinates is not loaded yet
        if (!this.coordinates.isInitialized() || !this.ground.mesh) {
            return;
        }

        const point = this.getGeolocation(event);
        this.routingEditor.addRoutingPoint(point, this.coordinates, this.scene);
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
        if (this.ground.type === "default" && !this.ground.initialized) {
            this.ground.initialize(this.coordinates);
            this.ground.mesh.name = "ground";
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
        this.adc.update(this.coordinates, world.autoDrivingCar);
        this.ground.update(world, this.coordinates, this.scene);
        this.planningTrajectory.update(world, world.planningData, this.coordinates, this.scene);
        this.perceptionObstacles.update(world, this.coordinates, this.scene);
        this.decision.update(world, this.coordinates, this.scene);
        this.prediction.update(world, this.coordinates, this.scene);
        this.updateRouting(world.routingTime, world.routePath);
        this.gnss.update(world, this.coordinates, this.scene);

        if (this.planningAdc &&
            world.planningTrajectory && world.planningTrajectory.length) {
            this.planningAdc.update(this.coordinates, world.planningTrajectory[0]);
        }
    }

    updateRouting(routingTime, routePath) {
        this.routing.update(routingTime, routePath, this.coordinates, this.scene);
    }

    updateGroundImage(mapName) {
        this.ground.updateImage(mapName);
    }

    updateGroundMetadata(serverUrl, mapInfo) {
        this.ground.initialize(serverUrl, mapInfo);
    }

    updateMap(newData, removeOldMap = false) {
        if (removeOldMap) {
            this.map.removeAllElements(this.scene);
        }
        this.map.appendMapData(newData, this.coordinates, this.scene);
    }

    updatePointCloud(pointCloud) {
        if (!this.coordinates.isInitialized() || !this.adc.mesh) {
            return;
        }
        this.pointCloud.update(pointCloud, this.adc.mesh);
    }

    updateMapIndex(hash, elementIds, radius) {
        if (!this.routingEditor.isInEditingMode() ||
             this.routingEditor.EDITING_MAP_RADIUS === radius) {
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
            0);

        vector.unproject(this.camera);

        const direction = vector.sub(this.camera.position).normalize();
        const distance = -this.camera.position.z / direction.z;
        const pos = this.camera.position.clone().add(direction.multiplyScalar(distance));
        const geo = this.coordinates.applyOffset(pos, true);

        return geo;
    }
}

const RENDERER = new Renderer();

export default RENDERER;
