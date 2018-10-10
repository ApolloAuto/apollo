import { observable, action, computed, extendObservable } from "mobx";

import PARAMETERS from "store/config/parameters.yml";
import MENU_DATA from "store/config/MenuData";

export default class Options {
    constructor() {
        this.cameraAngleNames = null;
        this.mainSideBarOptions = [
            "showTasks",
            "showModuleController",
            "showMenu",
            "showRouteEditingBar",
            "showDataRecorder",
        ];
        this.secondarySideBarOptions = ["showPOI", "enableAudioCapture"];

        // Set options and their default values from PARAMETERS.options
        const options = {};
        for (const name in PARAMETERS.options) {
            let defaultValue = PARAMETERS.options[name].default;
            if (OFFLINE_PLAYBACK && name === "showTasks") {
                defaultValue = false;
            }
            options[name] = defaultValue;
        }
        extendObservable(this, options);

        // Define toggles to hide in layer menu. These include PncMonitor
        // toggles, which are visible only when PNC Monitor is on.
        const togglesToHide = {
            perceptionPointCloud: OFFLINE_PLAYBACK,
            perceptionLaneMarker: OFFLINE_PLAYBACK,
            planningCar: OFFLINE_PLAYBACK,
        };
        for (const name in PARAMETERS.options) {
            const option = PARAMETERS.options[name];
            if (option.forPncMonitor) {
                togglesToHide[option.menuId] = true;
            }
        }
        this.togglesToHide = observable(togglesToHide);
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

    @action toggle(option) {
        this[option] = !this[option];

        // Disable other mutually exclusive options
        if (this[option] && this.mainSideBarOptions.includes(option)) {
            for (const other of this.mainSideBarOptions) {
                if (other !== option) {
                    this[other] = false;
                }
            }
        }

        if (option === "showPNCMonitor") {
            for (const name in PARAMETERS.options) {
                if (PARAMETERS.options[name].forPncMonitor) {
                    this.togglesToHide[PARAMETERS.options[name].menuId] = !this[option];
                }
            }
        }
    }

    @action setPncMonitorOptions(value) {
        for (const name in PARAMETERS.options) {
            if (PARAMETERS.options[name].forPncMonitor) {
                this[name] = value;
            }
        }
    }

    isSideBarButtonDisabled(option, enableHMIButtonsOnly, inNavigationMode) {
        if (!this.mainSideBarOptions.includes(option) &&
            !this.secondarySideBarOptions.includes(option)) {
            console.warn(`Disable logic for ${option} is not defined, return false.`);
            return false;
        }

        if (option === "showTasks" ||
            option === "showModuleController" ||
            option === "enableAudioCapture"
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
        }

        const currentIndex = this.cameraAngleNames.findIndex(name => name === this.cameraAngle);
        const nextIndex = (currentIndex + 1) % this.cameraAngleNames.length;
        this.selectCamera(this.cameraAngleNames[nextIndex]);
    }

    @action selectCamera(angleName) {
        this.cameraAngle = angleName;
    }
}
