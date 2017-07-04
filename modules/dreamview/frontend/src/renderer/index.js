import * as THREE from "three";

import PARAMETERS from "store/config/parameters.yml";
import Coordinates from "renderer/coordinates";
import AutoDrivingCar from "renderer/adc";
import Ground from "renderer/ground";
import PlanningTrajectory from "renderer/trajectory.js";

class Renderer {
    constructor() {
        this.coordinates = new Coordinates();
        this.renderer = new THREE.WebGLRenderer({
            preserveDrawingBuffer: true
        });
        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0x031C31);

        // Camera
        this.viewAngle = PARAMETERS.camera.defaults.viewAngle;
        this.viewDistance = (
            PARAMETERS.camera.laneWidth *
            PARAMETERS.camera.laneWidthToViewDistanceRatio);
        this.camera = new THREE.PerspectiveCamera(
            PARAMETERS.camera.defaults.fov,
            window.innerWidth / window.innerHeight,
            PARAMETERS.camera.defaults.near,
            PARAMETERS.camera.defaults.far
        );
        this.scene.add(this.camera);

        // The ground.
        this.ground = new Ground();

        // The main autonomous driving car.
        this.adc = new AutoDrivingCar();
        // The mesh this.adc.mesh is not added to the scene because it
        // takes time to load. It will be added later on.
        this.adcMeshAddedToScene = false;

        // The planning tranjectory.
        this.planningTrajectory = new PlanningTrajectory();

        // Stores the current rendering loop handle.
        this.renderingLoop = null;
    }

    initialize(canvasId, width, height) {
        this.updateDimension(width, height);
        this.renderer.setPixelRatio(window.devicePixelRatio);

        const container = document.getElementById(canvasId);
        container.appendChild(this.renderer.domElement);

        const ambient = new THREE.AmbientLight(0x444444);
        const directionalLight = new THREE.DirectionalLight(0xffeedd);
        directionalLight.position.set(0, 0, 1).normalize();

        // Hack fix orbit control plugin
        //
        // TODO maybe implement this?

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

    adjustCameraWithTarget(target) {
        // TODO Add more views.
        const deltaX = (this.viewDistance * Math.cos(target.rotation.y)
            * Math.cos(this.viewAngle));
        const deltaY = (this.viewDistance * Math.sin(target.rotation.y)
            * Math.cos(this.viewAngle));
        const deltaZ = this.viewDistance * Math.sin(this.viewAngle);

        this.camera.position.x = target.position.x - deltaX;
        this.camera.position.y = target.position.y - deltaY;
        this.camera.position.z = target.position.z + deltaZ;
        this.camera.up.set(0, 0, 1);
        this.camera.lookAt({
            x: target.position.x + 2 * deltaX,
            y: target.position.y + 2 * deltaY,
            z: 0
        });
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
            this.scene.add(this.adc.mesh);
        }

        // Upon the first time in render() it sees ground mesh loaded,
        // added it to the scene.
        if (!this.ground.initialized) {
            this.ground.initialize(this.coordinates);
            this.scene.add(this.ground.mesh);
        }

        this.adjustCameraWithTarget(this.adc.mesh);
        this.renderer.render(this.scene, this.camera);
    }

    animate() {
        this.renderingLoop = requestAnimationFrame(() => {
            this.animate();
        });
        this.render();
    }

    startAnimate() {
        if (!this.renderingLoop) {
            this.animate();
        }
    }

    stopAnimate() {
        if (this.renderingLoop) {
            cancelAnimationFrame(this.renderingLoop);
            this.renderingLoop = null;
        }
    }

    updateWorld(world) {
        this.adc.update(world, this.coordinates);
        this.planningTrajectory.update(world, this.coordinates, this.scene);
    }
}

const RENDERER = new Renderer();

export default RENDERER;
