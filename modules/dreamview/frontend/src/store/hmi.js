import { observable, action, computed } from "mobx";

import WS from "store/websocket";

export default class HMI {

    modes = {};
    @observable currentMode = 'none';

    vehicles = [];
    @observable currentVehicle = 'none';
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
    @observable hardwareStatus = observable.map();
    @observable enableStartAuto = false;

    displayName = {};
    utmZoneId = 10;

    @observable dockerImage = 'unknown';

    @action initialize(config) {
        if (config.dockerImage) {
            this.dockerImage = config.dockerImage;
        }
        if (config.modes) {
            this.modes = config.modes;
        }
        if (config.utmZoneId) {
            this.utmZoneId = config.utmZoneId;
        }
        this.vehicles = Object.keys(config.availableVehicles).sort()
            .map(name => {
                return name;
            });
        this.maps = Object.keys(config.availableMaps).sort()
            .map(name => {
                return name;
            });

        Object.keys(config.modules).forEach(key => {
            this.moduleStatus.set(key, false);
            this.displayName[key] = config.modules[key].displayName;
        });
        Object.keys(config.hardware).forEach(key => {
            this.hardwareStatus.set(key, 'NOT_READY');
            this.displayName[key] = config.hardware[key].displayName;
        });
    }

    @action updateStatus(newStatus) {
        if (newStatus.currentMode) {
            this.currentMode = newStatus.currentMode;
        }
        if (newStatus.currentMap) {
            this.currentMap = newStatus.currentMap;
        }
        if (newStatus.currentVehicle) {
            this.currentVehicle = newStatus.currentVehicle;
        }
        if (newStatus.systemStatus) {
            if (newStatus.systemStatus.modules) {
                for (const key in newStatus.systemStatus.modules) {
                    this.moduleStatus.set(key,
                        newStatus.systemStatus.modules[key].processStatus.running);
                }
            }
            if (newStatus.systemStatus.hardware) {
                for (const key in newStatus.systemStatus.hardware) {
                    this.hardwareStatus.set(key, newStatus.systemStatus.hardware[key].summary);
                }
            }
        }
    }

    @action update(world) {
        this.enableStartAuto = world.engageAdvice === "READY_TO_ENGAGE";
    }

    updateVehicleParam(vehicleParam) {
        this.vehicleParam = vehicleParam;
    }

    @action toggleModule(id) {
        this.moduleStatus.set(id, !this.moduleStatus.get(id));
        const command = this.moduleStatus.get(id) ? "start" : "stop";
        WS.executeModuleCommand(id, command);
    }

    @computed get showRTKCommands() {
        return this.currentMode === "RTK Record / Replay";
    }

    @computed get inNavigationMode() {
        return this.currentMode === "Navigation";
    }
}
