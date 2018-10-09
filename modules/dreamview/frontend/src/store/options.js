import { observable, action, computed, extendObservable } from "mobx";

import PARAMETERS from "store/config/parameters.yml";
import MENU_DATA from "store/config/MenuData";

export default class Options {
    constructor() {
        const options = {};
        Object.keys(PARAMETERS.options.defaults).forEach(option => {
            let defaultValue = PARAMETERS.options.defaults[option];
            if (OFFLINE_PLAYBACK && option === "showTasks") {
                defaultValue = false;
            }
            options[option] = defaultValue;
        });
        extendObservable(this, options);

        this.cameraAngleNames = null;
        this.mainSideBarOptions = [
            "showTasks",
            "showModuleController",
            "showMenu",
            "showRouteEditingBar",
            "showDataRecorder",
        ];
        this.secondarySideBarOptions = ["showPOI", "enableAudioCapture"];
        this.hideOptionToggle = observable({
            planningCar: true,
            planningQpOptimizer: true,
            planningDpOptimizer: true,
            planningReference: true,
            perceptionPointCloud: OFFLINE_PLAYBACK,
            perceptionLaneMarker: OFFLINE_PLAYBACK,
        });
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
            Object.keys(this.hideOptionToggle).map((toggle) => {
                if (toggle.startsWith("planning")) {
                    this.hideOptionToggle[toggle] = !this[option];
                }
            });
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
