import { observable, action } from "mobx";

import WS from "store/websocket";

export default class HMI {
    @observable modes = [];
    @observable currentMode = 'none';

    @observable vehicles = [];
    @observable currentVehicle = 'none';

    @observable maps = [];
    @observable currentMap = 'none';

    @observable modules = observable.map();
    @observable hardware = observable.map();

    displayName = {};

    @action initialize(config) {
        this.modes = Object.keys(config.modes).sort()
            .map(name => {
                return name;
            });
        this.vehicles = Object.keys(config.availableVehicles).sort()
            .map(name => {
                return name;
            });
        this.maps = Object.keys(config.availableMaps).sort()
            .map(name => {
                return name;
            });

        Object.keys(config.modules).forEach(key => {
            this.modules.set(key, false);
            this.displayName[key] = config.modules[key].displayName;
        });
        Object.keys(config.hardware).forEach(key => {
            this.hardware.set(key, 'NOT_READY');
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
                    this.modules.set(key, newStatus.systemStatus.modules[key].processRunning);
                }
            }
            if (newStatus.systemStatus.hardware) {
                for (const key in newStatus.systemStatus.hardware) {
                    this.hardware.set(key, newStatus.systemStatus.hardware[key].status);
                }
            }
        }
    }

    @action toggleModule(id) {
        this.modules.set(id, !this.modules.get(id));
        const command = this.modules.get(id) ? "start" : "stop";
        WS.executeModuleCommand(id, command);
    }
}
