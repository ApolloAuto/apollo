import * as THREE from "three";
import "imports-loader?THREE=three!three/examples/js/controls/OrbitControls.js";

import PARAMETERS from "store/config/parameters.yml";
import Coordinates from "renderer/coordinates";
import AutoDrivingCar from "renderer/adc";
import Ground from "renderer/ground";
import Map from "renderer/map";
import PlanningTrajectory from "renderer/trajectory.js";
import PerceptionObstacles from "renderer/obstacles.js";
import Decision from "renderer/decision.js";
import Prediction from "renderer/prediction.js";
import Routing from "renderer/routing.js";

class Renderer {
    constructor() {
        this.coordinates = new Coordinates();
        this.renderer = new THREE.WebGLRenderer({
            preserveDrawingBuffer: true, antialias: true
        });
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x031C31);

        // The ground. (grid for now)
        this.ground = new Ground();

        // The map.
        this.map = new Map();

        // The main autonomous driving car.
        this.adc = new AutoDrivingCar();
        // The mesh this.adc.mesh is not added to the scene because it
        // takes time to load. It will be added later on.
        this.adcMeshAddedToScene = false;

        // The planning tranjectory.
        this.planningTrajectory = new PlanningTrajectory();

        // The perception obstacles.
        this.perceptionObstacles = new PerceptionObstacles();

        // The decision.
        this.decision = new Decision();

        // The prediction.
        this.prediction = new Prediction();

        // The routing.
        this.routing = new Routing();
    }

    initialize(canvasId, width, height, options) {
        this.options = options;

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

        this.scene.add(ambient);
        this.scene.add(directionalLight);

        // TODO maybe add sanity check.

        // Actually start the animation.
        this.animate();
    }

    maybeInitializeOffest(x, y) {
        if (!this.coordinates.isInitialized()) {
            this.coordinates.initialize(x, y);
        }
    }

    updateDimension(width, height) {
        this.camera.aspect = width / height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(width, height);
    }

    enableOrbitControls(){
        this.controls.enabled = true;
        this.controls.enableRotate = false;
        this.controls.reset();
        this.controls.minDistance = 10;
        this.controls.maxDistance = 2000;

        const carPosition = this.adc.mesh.position;
        this.camera.position.set(carPosition.x, carPosition.y, 50);
        this.camera.up.set(0, 1, 0);
        this.camera.lookAt(carPosition.x, carPosition.y, 0);
    }

    adjustCamera(target, pov) {
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
            this.camera.up.set(0, 1, 0);
            this.camera.lookAt({
                x: target.position.x,
                y: target.position.y + deltaY,
                z: 0
            });

            this.controls.enabled = false;
            break;
        case "Map":
            if (!this.controls.enabled){
                this.enableOrbitControls();
            }else {
                this.camera.up.set(0, 1, 0);
                this.camera.lookAt(this.camera.position.x, this.camera.position.y, 0);
            }
            break;
        }
        this.camera.updateProjectionMatrix();
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

        // Upon the first time in render() it sees car mesh loaded,
        // added it to the scene.
        if (!this.adcMeshAddedToScene) {
            this.adcMeshAddedToScene = true;
            this.adc.mesh.name = "adc";
            this.scene.add(this.adc.mesh);
        }

        // Upon the first time in render() it sees ground mesh loaded,
        // added it to the scene.
        if (!this.ground.initialized) {
            this.ground.initialize(this.coordinates);
            this.ground.mesh.name = "ground";
            this.scene.add(this.ground.mesh);
        }

        this.adjustCamera(this.adc.mesh, this.options.cameraAngle);
        this.renderer.render(this.scene, this.camera);
    }

    animate() {
        requestAnimationFrame(() => {
            this.animate();
        });
        this.render();
    }

    updateWorld(world) {
        this.adc.update(world, this.coordinates);
        this.planningTrajectory.update(world, this.coordinates, this.scene);
        this.perceptionObstacles.update(world, this.coordinates, this.scene);
        this.decision.update(world, this.coordinates, this.scene);
        this.prediction.update(world, this.coordinates, this.scene);
        this.routing.update(world, this.coordinates, this.scene);
    }

    updateMap(newData) {
        this.map.appendMapData(newData, this.coordinates, this.scene);
    }

    updateMapIndex(hash, elementIds) {
        this.map.updateIndex(hash, elementIds, this.scene);
    }
}

const RENDERER = new Renderer();

export default RENDERER;
