import { observable, action } from "mobx";

export default class HMI {
    @observable maps = [];
    @observable currentMap = 'none';
    @observable vehicles = [];
    @observable currentVehicle = 'none';
    @observable status = {};

    @action updateMaps(availableMaps) {
        this.maps = Object.keys(availableMaps).sort()
            .map(name => {
                return name;
            });
    }

    @action updateVehicles(availableVehicles) {
        this.vehicles = Object.keys(availableVehicles).sort()
            .map(name => {
                return name;
            });
    }

    @action updateConfig(newConfig) {
        console.log("config:", newConfig);
        this.updateMaps(newConfig.availableMaps);
        this.updateVehicles(newConfig.availableVehicles);
    }

    @action updateStatus(newStatus) {
        console.log("status:", newStatus);
        this.status = newStatus;
        if (newStatus.currentMap) {
            this.currentMap = newStatus.currentMap;
        }
    }
}
