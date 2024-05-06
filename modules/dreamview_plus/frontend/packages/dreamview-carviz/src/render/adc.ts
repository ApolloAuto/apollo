import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { isNumber } from 'lodash';
import * as THREE from 'three';
import carMaterial from '../../assets/models/car.mtl';
import carObject from '../../assets/models/car.obj';
import { disposeMesh } from '../utils/common';

const defaultVehicleSize = {
    height: 1.48,
    width: 2.11,
    length: 4.933,
};
export default class Adc {
    public adc;

    public shadowAdc;

    public planningAdc;

    public isInitialized;

    private scene;

    public vehicleParam;

    private option;

    public autoDrivingCar;

    private coordinates;

    private boudingBox;

    private center;

    constructor(scene, option, coordinates) {
        this.scene = scene;
        this.option = option;
        this.adc = null;
        this.shadowAdc = null;
        this.planningAdc = null;
        this.isInitialized = false;
        this.autoDrivingCar = null;
        this.vehicleParam = {
            frontEdgeToCenter: 3.89,
            backEdgeToCenter: 1.043,
            leftEdgeToCenter: 1.055,
            rightEdgeToCenter: 1.055,
            height: 1.48,
            width: 2.11,
            length: 4.933,
            steerRatio: 16,
            wheelBase: 2.8448,
        };
        this.coordinates = coordinates;
        this.init();
    }

    private init() {
        const mtlLoader = new MTLLoader();
        const objLoader = new OBJLoader();
        objLoader.load(carObject, (object) => {
            this.planningAdc = object.clone();
            this.planningAdc.name = 'planningAdc';
            this.planningAdc.rotation.x = Math.PI / 2;
            this.planningAdc.visible = false;
            this.shadowAdc = object.clone();
            this.shadowAdc.name = 'shadowAdc';
            this.shadowAdc.rotation.x = Math.PI / 2;
            this.shadowAdc.visible = false;

            this.scene.add(this.planningAdc);
            this.scene.add(this.shadowAdc);
        });
        mtlLoader.load(carMaterial, (material) => {
            material.preload();
            objLoader.setMaterials(material);
            objLoader.load(carObject, (object) => {
                this.adc = object;
                this.adc.name = 'adc';
                this.adc.rotation.x = Math.PI / 2;
                this.adc.visible = false;
                this.scene.add(this.adc);
            });
        });
    }

    public update(pos, name) {
        if (!this[name] || !pos || !isNumber(pos.positionX) || !isNumber(pos.positionY)) {
            return;
        }

        if (!this.coordinates.isInitialized()) {
            this.coordinates.initialize(pos.positionX, pos.positionY);
        }

        if (this[name] && pos) {
            const { positionX, positionY, heading } = pos;
            const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
            if (!position) {
                return;
            }
            this[name].position.set(position.x, position.y, 0);
            this[name].rotation.y = heading;
            if (name === 'adc') {
                this.adc.visible = !pos.boudingBox;
                this.autoDrivingCar = pos;

                disposeMesh(this.center);
                this.scene.remove(this.center);
                disposeMesh(this.boudingBox);
                this.scene.remove(this.boudingBox);
                if (pos.boudingBox) {
                    const cetenrGeometry = new THREE.BufferGeometry();
                    cetenrGeometry.setAttribute(
                        'position',
                        new THREE.BufferAttribute(
                            new Float32Array([position.x, position.y, this.vehicleParam.height / 2]),
                            3,
                        ),
                    );
                    const centerMaterial = new THREE.PointsMaterial({
                        color: '#FF0000',
                        size: 0.5, // 点对象像素尺寸
                    });
                    const center = new THREE.Points(cetenrGeometry, centerMaterial);
                    // disposeMesh(this.center);
                    // this.scene.remove(this.center);
                    this.center = center;
                    this.scene.add(center);
                    const width = this.vehicleParam.leftEdgeToCenter + this.vehicleParam.rightEdgeToCenter;
                    const length = this.vehicleParam.backEdgeToCenter + this.vehicleParam.frontEdgeToCenter;
                    // console.log(`左距:${this.vehicleParam.leftEdgeToCenter},右距:${this.vehicleParam.rightEdgeToCenter}`);
                    // console.log(`前距:${this.vehicleParam.frontEdgeToCenter},后距:${this.vehicleParam.backEdgeToCenter}`);
                    // 将数组分成每三个元素一组
                    // const positions = geometry.attributes.position.array;
                    // const groupedVertices = [];
                    // for (let i = 0; i < positions.length; i += 3) {
                    //     groupedVertices.push(positions.slice(i, i + 3));
                    // }
                    // // 遍历每个组并输出
                    // groupedVertices.forEach((group, index) => {
                    //     console.log(`Group ${index + 1}:`);
                    //     group.forEach((vertex, i) => {
                    //         console.log(`Vertex ${i + 1} position: ${vertex}`);
                    //     });
                    // });
                    const geometry = new THREE.BoxGeometry(length, width, this.vehicleParam.height);
                    const material = new THREE.MeshBasicMaterial({ color: 0x00ff00, wireframe: true });
                    const boundingBox = new THREE.Mesh(geometry, material);
                    const distance = (this.vehicleParam.frontEdgeToCenter - this.vehicleParam.backEdgeToCenter) / 2;
                    boundingBox.position.set(
                        position.x + Math.cos(pos.heading) * distance,
                        position.y + Math.sin(pos.heading) * distance,
                        this.vehicleParam.height / 2,
                    );
                    boundingBox.rotation.z = pos.heading;
                    // disposeMesh(this.boudingBox);
                    // this.scene.remove(this.boudingBox);
                    this.boudingBox = boundingBox;
                    this.scene.add(boundingBox);
                }
            }
            if (name === 'shadowAdc') {
                this.shadowAdc.visible = this.option.layerOption.Position.shadow;
            }
            if (name === 'planningAdc') {
                this.planningAdc.visible = this.option.layerOption.Planning.planningCar;
            }
        }
    }

    public updateOffset(pos, name) {
        if (!this[name] || !pos || !isNumber(pos.positionX) || !isNumber(pos.positionY)) {
            return;
        }

        if (!this.coordinates.isInitialized()) {
            this.coordinates.initialize(pos.positionX, pos.positionY);
        }

        if (this[name] && pos) {
            const { positionX, positionY, heading } = pos;
            const position = this.coordinates.applyOffset({ x: positionX, y: positionY });
            if (!position) {
                return;
            }

            this[name].position.set(position.x, position.y, 0);
            this[name].rotation.y = heading;
            if (name === 'adc') {
                this.adc.visible = false;
            }
            if (name === 'shadowAdc') {
                this.shadowAdc.visible = false;
            }
            if (name === 'planningAdc') {
                this.planningAdc.visible = false;
            }
        }
    }

    updateVehicleParam(params) {
        this.vehicleParam = params;
        this.resizeCarSize(
            this.vehicleParam.length / defaultVehicleSize.length,
            this.vehicleParam.height / defaultVehicleSize.height,
            this.vehicleParam.width / defaultVehicleSize.width,
        );
    }

    resizeCarSize(x, y, z) {
        this.shadowAdc?.scale?.set(x, y, z);
        this.adc?.scale?.set(x, y, z);
        this.planningAdc?.scale?.set(x, y, z);
    }
}
