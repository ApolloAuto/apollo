import { observable, action, computed } from "mobx";

import WS from "store/websocket";
import UTTERANCE from "store/utterance";
import RENDERER from "renderer";

export default class HMI {
    modes = [];
    @observable currentMode = 'none';

    vehicles = [];
    @observable currentVehicle = 'none';
    defaultVehicleSize = {
        height: 1.48,
        width: 2.11,
        length: 4.933,
    };
    vehicleParam = {
        frontEdgeToCenter: 3.89,
        backEdgeToCenter: 1.04,
        leftEdgeToCenter: 1.055,
        rightEdgeToCenter: 1.055,
        height: 1.48,
        width: 2.11,
        length: 4.933,
        steerRatio: 16,
    };

    maps = [];
    @observable currentMap = 'none';

    @observable moduleStatus = observable.map();
    @observable componentStatus = observable.map();
    @observable enableStartAuto = false;

    displayName = {};
    utmZoneId = 10;

    @observable dockerImage = 'unknown';

    @observable isCoDriver = false;

    @observable isMute = false;

    @action toggleCoDriverFlag() {
        this.isCoDriver = !this.isCoDriver;
    }

    @action toggleMuteFlag() {
        this.isMute = !this.isMute;
        UTTERANCE.setMute(this.isMute);
    }

    @action updateStatus(newStatus) {
        if (newStatus.dockerImage) {
            this.dockerImage = newStatus.dockerImage;
        }
        if (newStatus.utmZoneId) {
            this.utmZoneId = newStatus.utmZoneId;
        }

        if (newStatus.modes) {
            this.modes = newStatus.modes.sort();
        }
        if (newStatus.currentMode) {
            this.currentMode = newStatus.currentMode;
        }

        if (newStatus.maps) {
            this.maps = newStatus.maps.sort();
        }
        if (newStatus.currentMap) {
            this.currentMap = newStatus.currentMap;
        }

        if (newStatus.vehicles) {
            this.vehicles = newStatus.vehicles.sort();
        }
        if (newStatus.currentVehicle) {
            this.currentVehicle = newStatus.currentVehicle;
        }

        if (newStatus.modules) {
            const newKeyList = JSON.stringify(Object.keys(newStatus.modules).sort());
            const curKeyList = JSON.stringify(this.moduleStatus.keys().sort());
            if (newKeyList !== curKeyList) {
                this.moduleStatus.clear();
            }
            for (const key in newStatus.modules) {
                this.moduleStatus.set(key, newStatus.modules[key]);
            }
        }

        if (newStatus.monitoredComponents) {
            const newKeyList = JSON.stringify(Object.keys(newStatus.monitoredComponents).sort());
            const curKeyList = JSON.stringify(this.componentStatus.keys().sort());
            if (newKeyList !== curKeyList) {
                this.componentStatus.clear();
            }
            for (const key in newStatus.monitoredComponents) {
                this.componentStatus.set(key, newStatus.monitoredComponents[key]);
            }
        }

        if (typeof newStatus.passengerMsg === "string") {
            UTTERANCE.speakRepeatedly(newStatus.passengerMsg);
        }
    }

    @action update(world) {
        this.enableStartAuto = world.engageAdvice === "READY_TO_ENGAGE";
    }

    updateVehicleParam(vehicleParam) {
        this.vehicleParam = vehicleParam;
        RENDERER.adc.resizeCarScale(this.vehicleParam.length / this.defaultVehicleSize.length,
            this.vehicleParam.width / this.defaultVehicleSize.width,
            this.vehicleParam.height / this.defaultVehicleSize.height);
    }

    @action toggleModule(id) {
        this.moduleStatus.set(id, !this.moduleStatus.get(id));
        const command = this.moduleStatus.get(id) ? "START_MODULE" : "STOP_MODULE";
        WS.executeModuleCommand(id, command);
    }

    @computed get inNavigationMode() {
        return this.currentMode === "Navigation";
    }
}
