import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls';
import View from './render/view';
import Map from './render/map';
import Adc from './render/adc';
import Obstacles from './render/obstacles';
import Text from './render/text';
import PointCloud from './render/pointCloud';
import Routing from './render/routing';
import Decision from './render/decision';
import Prediction from './render/prediction';
import Planning from './render/planning';
import Gps from './render/gps';
import Option from './option';
import Coordinates from './render/coordinates';
import { cameraParams } from './constant/common';

export default class Carviz {
    private canvasId;

    private canvasDom;

    private width;

    private height;

    public scene;

    public renderer;

    public camera;

    public controls;

    public option;

    public view;

    public text;

    public map;

    public adc;

    public obstacles;

    public pointCloud;

    public routing;

    public decision;

    public prediction;

    public planning;

    public gps;

    private initialized;

    private coordinates;

    constructor(id) {
        this.canvasId = id;
        this.initialized = false;
    }

    render() {
        if (this.initialized) {
            this.view?.setView();
            this.renderer.render(this.scene, this.camera);
        }
    }

    updateDimention() {
        this.camera.aspect = this.width / this.height;
        this.camera.updateProjectionMatrix();
        this.renderer.setSize(this.width, this.height);
        this.render();
    }

    initDom() {
        this.canvasDom = document.getElementById(this.canvasId);
        if (!this.canvasDom || !this.canvasId) {
            throw new Error('no canvas container');
        }
        this.width = this.canvasDom.clientWidth;
        this.height = this.canvasDom.clientHeight;
    }

    initThree() {
        this.scene = new THREE.Scene();

        this.renderer = new THREE.WebGLRenderer({
            alpha: true,
            antialias: true,
        });
        this.renderer.setPixelRatio(window.devicePixelRatio);
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setClearColor(0x0f1014);
        this.canvasDom.appendChild(this.renderer.domElement);

        this.camera = new THREE.PerspectiveCamera(
            cameraParams.Default.fov,
            this.width / this.height,
            cameraParams.Default.near,
            cameraParams.Default.far,
        );
        this.camera.up.set(0, 0, 1);

        const light = new THREE.DirectionalLight(0xffeedd);
        light.position.set(0, 0, 10);
        this.scene.add(light);

        this.controls = new OrbitControls(this.camera, this.renderer.domElement);
        this.controls.enabled = false;
        this.controls.keys = {
            LEFT: 'ArrowLeft',
            UP: 'ArrowUp',
            RIGHT: 'ArrowRight',
            BOTTOM: 'ArrowDown',
        };
        this.controls.listenToKeyEvents(window);
        this.controls.addEventListener('change', () => {
            this.view?.setView();
            this.render();
        });
        this.controls.keys = {
            LEFT: 'ArrowLeft',
            UP: 'ArrowUp',
            RIGHT: 'ArrowRight',
            BOTTOM: 'ArrowDown',
        };
        this.controls.mouseButtons = {
            LEFT: THREE.MOUSE.ROTATE,
            MIDDLE: THREE.MOUSE.DOLLY,
            RIGHT: THREE.MOUSE.PAN,
        };
        this.updateDimention();
        const resizeObserver = new ResizeObserver(() => {
            this.width = this.canvasDom?.clientWidth;
            this.height = this.canvasDom?.clientHeight;
            this.updateDimention();
        });
        resizeObserver.observe(this.canvasDom);
        this.render();
    }

    initModule() {
        this.coordinates = new Coordinates();
        this.option = new Option();
        this.adc = new Adc(this.scene, this.option, this.coordinates);
        this.view = new View(this.camera, this.controls, this.adc);
        this.text = new Text(this.camera);
        this.map = new Map(this.scene, this.text, this.option, this.coordinates);
        this.obstacles = new Obstacles(this.scene, this.view, this.text, this.option, this.coordinates);
        this.pointCloud = new PointCloud(this.scene, this.adc, this.option);
        this.routing = new Routing(this.scene, this.option, this.coordinates);
        this.decision = new Decision(this.scene, this.option, this.coordinates);
        this.prediction = new Prediction(this.scene, this.option, this.coordinates);
        this.planning = new Planning(this.scene, this.option, this.coordinates);
        this.gps = new Gps(this.scene, this.adc, this.option, this.coordinates);
    }

    init() {
        this.initDom();
        this.initThree();
        this.initModule();
        this.initialized = true;
    }

    updateData(datas) {
        if (datas instanceof Object) {
            const dataKeys = Object.keys(datas);
            dataKeys.forEach((key) => {
                const data = datas[key];
                switch (key) {
                    case 'autoDrivingCar':
                        this.adc.update(data, 'adc');
                        break;
                    case 'map':
                        this.map.update(data, false);
                        break;
                    case 'pointCloud':
                        this.pointCloud.update(data);
                        break;
                    case 'shadowLocalization':
                        this.adc.update(data, 'shadowAdc');
                        break;
                    case 'planningData':
                        this.adc.update(data.initPoint?.pathPoint, 'planningAdc');
                        break;
                    case 'mainDecision':
                        this.decision.updateMainDecision(data);
                        break;
                    case 'object':
                        this.decision.updateObstacleDecision(data);
                        this.obstacles.update(data, datas.sensorMeasurements, datas.autoDrivingCar);
                        this.prediction.update(data);
                        break;
                    case 'gps':
                        this.gps.update(data);
                        break;
                    case 'planningTrajectory':
                        this.planning.update(data, datas.planningData, datas.autoDrivingCar);
                        break;
                    case 'routePath':
                        this.routing.update(datas.routingTime, data);
                        break;
                    default:
                        break;
                }
            });
        }
    }

    removeAll() {
        this.map.dispose();
        this.obstacles.dispose();
        this.pointCloud.dispose();
        this.routing.dispose();
        this.decision.dispose();
        this.prediction.dispose();
        this.planning.dispose();
        this.gps.dispose();
    }

    removeMap() {
        this.map.dispose();
    }

    removeObstacles() {
        this.obstacles.dispose();
    }

    removePointCloud() {
        this.pointCloud.dispose();
    }

    removeRouting() {
        this.routing.dispose();
    }

    removeDecision() {
        this.decision.dispose();
    }

    removePrediction() {
        this.prediction.dispose();
    }

    removePlanning() {
        this.planning.dispose();
    }

    removeGps() {
        this.gps.dispose();
    }
}
