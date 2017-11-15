import { observable, action } from "mobx";

import WS from "store/websocket";

export default class HMI {

    modes = {};
    @observable currentMode = 'none';

    vehicles = [];
    @observable currentVehicle = 'none';

    maps = [];
    @observable currentMap = 'none';

    @observable moduleStatus = observable.map();
    @observable hardwareStatus = observable.map();

    displayName = {};

    @action initialize(config) {
        if (config.modes) {
            this.modes = config.modes;
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
                    this.moduleStatus.set(key, newStatus.systemStatus.modules[key].processRunning);
                }
            }
            if (newStatus.systemStatus.hardware) {
                for (const key in newStatus.systemStatus.hardware) {
                    this.hardwareStatus.set(key, newStatus.systemStatus.hardware[key].status);
                }
            }
        }
    }

    @action toggleModule(id) {
        this.moduleStatus.set(id, !this.moduleStatus.get(id));
        const command = this.moduleStatus.get(id) ? "start" : "stop";
        WS.executeModuleCommand(id, command);
    }
}
