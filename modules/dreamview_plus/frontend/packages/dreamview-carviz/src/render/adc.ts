import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { isNumber } from 'lodash';
import carMaterial from '../../assets/models/car.mtl';
import carObject from '../../assets/models/car.obj';

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
                this.adc.visible = this.option.layerOption.Position.localization;
                this.autoDrivingCar = pos;
            }
            if (name === 'shadowAdc') {
                this.shadowAdc.visible = this.option.layerOption.Position.shadow;
            }
            if (name === 'planningAdc') {
                this.planningAdc.visible = this.option.layerOption.Planning.planningCar;
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
        this.shadowAdc?.scale(x, y, z);
        this.adc?.scale(x, y, z);
        this.planningAdc?.scale(x, y, z);
    }
}
