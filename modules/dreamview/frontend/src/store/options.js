import { observable, action, computed, extendObservable, isComputed } from "mobx";

import _ from 'lodash';
import MENU_DATA from "store/config/MenuData";

export const MONITOR_MENU = Object.freeze({
    PNC_MONITOR: 'showPNCMonitor',
    DATA_COLLECTION_MONITOR: 'showDataCollectionMonitor',
    CONSOLE_TELEOP_MONITOR: 'showConsoleTeleopMonitor',
    CAR_TELEOP_MONITOR: 'showCarTeleopMonitor',
    CAMERA_PARAM: 'showCameraView',
});

export default class Options {
    // Toggles added by planning paths when pnc monitor is on
    @observable customizedToggles = observable.map();

    constructor() {
        this.cameraAngleNames = null;
        this.mainSideBarOptions = [
            "showTasks",
            "showModuleController",
            "showMenu",
            "showRouteEditingBar",
            "showDataRecorder",
        ];
        this.secondarySideBarOptions = ["showPOI"];

        // Set options and their default values from PARAMETERS.options
        this.resetOptions();

        // Define toggles to hide in layer menu. These include PncMonitor
        // toggles, which are visible only when PNC Monitor is on.
        const togglesToHide = {
            perceptionPointCloud: OFFLINE_PLAYBACK,
            perceptionLaneMarker: OFFLINE_PLAYBACK,
            planningCar: OFFLINE_PLAYBACK,
        };
        this.togglesToHide = observable(togglesToHide);
    }

    @action resetOptions() {
        const options = {};
        for (const name in PARAMETERS.options) {
            let defaultValue = PARAMETERS.options[name].default;
            if (OFFLINE_PLAYBACK && name === "showTasks") {
                defaultValue = false;
            }
            if (OFFLINE_PLAYBACK && name === "showPositionShadow") {
                defaultValue = true;
            }
            options[name] = defaultValue;
        }
        extendObservable(this, options);
    }

    @computed get showTools() {
        return this.showTasks ||
               this.showModuleController ||
               this.showMenu ||
               this.showPOI ||
               this.showDataRecorder;
    }

    @computed get showGeo() {
        return this.showRouteEditingBar ||
               this.cameraAngle === 'Map' ||
               this.cameraAngle === 'Overhead' ||
               this.cameraAngle === 'Monitor';
    }

    @computed get showMonitor() {
        for (const option of Object.values(MONITOR_MENU)) {
            if (this[option]) {
                return true;
            }
        }
        return false;
    }

    @computed get monitorName() {
        if (this.showConsoleTeleopMonitor) {
            return MONITOR_MENU.CONSOLE_TELEOP_MONITOR;
        } else if (this.showCarTeleopMonitor) {
            return MONITOR_MENU.CAR_TELEOP_MONITOR;
        } else if (this.showCameraView) {
            return MONITOR_MENU.CAMERA_PARAM;
        } else if (this.showDataCollectionMonitor) {
            return MONITOR_MENU.DATA_COLLECTION_MONITOR;
        } else if (this.showPNCMonitor) {
            return MONITOR_MENU.PNC_MONITOR;
        } else {
            return null;
        }
    }

    @computed get showCameraView() {
        return this.cameraAngle === "CameraView";
    }

    @action toggle(option, isCustomized) {
        if (isCustomized) {
            this.customizedToggles.set(option, !this.customizedToggles.get(option));
        } else {
            this[option] = !this[option];
        }

        // Disable other mutually exclusive options
        if (this[option] && this.mainSideBarOptions.includes(option)) {
            for (const other of this.mainSideBarOptions) {
                if (other !== option) {
                    this[other] = false;
                }
            }
        }
        const monitorOptions = new Set(Object.values(MONITOR_MENU));
        if (monitorOptions.has(option)) {
            for (const other of monitorOptions) {
                if (other !== option && !isComputed(this, other)) {
                    this[other] = false;
                }
            }
        }
    }

    @action setCustomizedToggles(toggles) {
        // Set additional toggle in observable map
        this.customizedToggles.clear();
        if (toggles) {
            this.customizedToggles.merge(toggles);
        }
    }

    isSideBarButtonDisabled(option, enableHMIButtonsOnly, inNavigationMode) {
        if (!this.mainSideBarOptions.includes(option) &&
            !this.secondarySideBarOptions.includes(option)) {
            console.warn(`Disable logic for ${option} is not defined, return false.`);
            return false;
        }

        if (option === "showTasks" ||
            option === "showModuleController"
        ) {
            return false;
        } else if (option === "showRouteEditingBar") {
            return enableHMIButtonsOnly || inNavigationMode;
        } else if (option === "showPOI") {
            return enableHMIButtonsOnly || this.showRouteEditingBar;
        } else {
            return enableHMIButtonsOnly;
        }
    }

    rotateCameraAngle() {
        if (!this.cameraAngleNames) {
            const cameraData = MENU_DATA.find(data => {
                return data.id === "camera";
            });

            this.cameraAngleNames = Object.values(cameraData.data);
            const shouldFilterCameraView = _.get(PARAMETERS, 'cameraAngle.hasCameraView', true);
            if (shouldFilterCameraView) {
                this.cameraAngleNames = this.cameraAngleNames.filter(name => name !== 'CameraView');
            }
        }

        const currentIndex = this.cameraAngleNames.findIndex(name => name === this.cameraAngle);
        const nextIndex = (currentIndex + 1) % this.cameraAngleNames.length;
        this.selectCamera(this.cameraAngleNames[nextIndex]);
    }

    @action selectCamera(angleName) {
        this.cameraAngle = angleName;
    }
}
