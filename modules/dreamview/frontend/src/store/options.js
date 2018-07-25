import { observable, action, computed } from "mobx";

import PARAMETERS from "store/config/parameters.yml";
import MENU_DATA from 'store/config/MenuData';

export default class Options {
    // Side Bar options
    @observable showModuleController = PARAMETERS.options.defaults.showModuleController;
    @observable showMenu = PARAMETERS.options.defaults.showMenu;
    @observable showRouteEditingBar = PARAMETERS.options.defaults.showRouteEditingBar;
    @observable showPOI = PARAMETERS.options.defaults.showPOI;
    @observable enableVoiceCommand = PARAMETERS.options.defaults.enableVoiceCommand;
    @observable showDataRecorder = PARAMETERS.options.defaults.showDataRecorder;
    @observable showVideo = PARAMETERS.options.defaults.showVideo;
    @observable showTasks =
        OFFLINE_PLAYBACK ? false : PARAMETERS.options.defaults.showTasks;


    // Layer Menu options
    @observable cameraAngle = PARAMETERS.options.defaults.cameraAngle;
    @observable showDecisionMain = PARAMETERS.options.defaults.showDecisionMain;
    @observable showDecisionObstacle = PARAMETERS.options.defaults.showDecisionObstacle;
    @observable showPlanning = PARAMETERS.options.defaults.showPlanning;
    @observable showPlanningCar = PARAMETERS.options.defaults.showPlanningCar;
    @observable showPlanningReference = PARAMETERS.options.defaults.showPlanningReference;
    @observable showPlanningDpOptimizer = PARAMETERS.options.defaults.showPlanningDpOptimizer;
    @observable showPlanningQpOptimizer = PARAMETERS.options.defaults.showPlanningQpOptimizer;
    @observable showRouting = PARAMETERS.options.defaults.showRouting;
    @observable showPerceptionLaneMarker = PARAMETERS.options.defaults.showPerceptionLaneMarker;
    @observable showPredictionMajor = PARAMETERS.options.defaults.showPredictionMajor;
    @observable showPredictionMinor = PARAMETERS.options.defaults.showPredictionMinor;
    @observable showObstaclesVehicle = PARAMETERS.options.defaults.showObstaclesVehicle;
    @observable showObstaclesPedestrian = PARAMETERS.options.defaults.showObstaclesPedestrian;
    @observable showObstaclesBicycle = PARAMETERS.options.defaults.showObstaclesBicycle;
    @observable showObstaclesUnknownMovable =
        PARAMETERS.options.defaults.showObstaclesUnknownMovable;
    @observable showObstaclesUnknownUnmovable =
        PARAMETERS.options.defaults.showObstaclesUnknownUnmovable;
    @observable showObstaclesUnknown =
        PARAMETERS.options.defaults.showObstaclesUnknown;
    @observable showObstaclesVirtual =
        PARAMETERS.options.defaults.showObstaclesVirtual;
    @observable showObstaclesCipv =
        PARAMETERS.options.defaults.showObstaclesCipv;
    @observable showObstaclesVelocity =
        PARAMETERS.options.defaults.showObstaclesVelocity;
    @observable showObstaclesHeading =
        PARAMETERS.options.defaults.showObstaclesHeading;
    @observable showObstaclesId =
        PARAMETERS.options.defaults.showObstaclesId;
    @observable showObstaclesInfo =
        PARAMETERS.options.defaults.showObstaclesInfo;
    @observable showPointCloud = PARAMETERS.options.defaults.showPointCloud;
    @observable showPositionGps = PARAMETERS.options.defaults.showPositionGps;
    @observable showPositionLocalization = PARAMETERS.options.defaults.showPositionLocalization;
    @observable showMapCrosswalk = PARAMETERS.options.defaults.showMapCrosswalk;
    @observable showMapClearArea = PARAMETERS.options.defaults.showMapClearArea;
    @observable showMapJunction = PARAMETERS.options.defaults.showMapJunction;
    @observable showMapLane = PARAMETERS.options.defaults.showMapLane;
    @observable showMapRoad = PARAMETERS.options.defaults.showMapRoad;
    @observable showMapSignal = PARAMETERS.options.defaults.showMapSignal;
    @observable showMapStopSign = PARAMETERS.options.defaults.showMapStopSign;

    // Others
    @observable showPNCMonitor = PARAMETERS.options.defaults.showPNCMonitor;
    @observable simControlEnabled = PARAMETERS.options.defaults.enableSimControl;
    @observable tasksPanelLocked = false;

    @observable hideOptionToggle = {
        'planningCar': true,
        'planningQpOptimizer': true,
        'planningDpOptimizer': true,
        'planningReference': true,
        'perceptionPointCloud': OFFLINE_PLAYBACK,
        'perceptionLaneMarker': OFFLINE_PLAYBACK,
    };

    cameraAngleNames = null;
    mainSideBarOptions = ['showTasks', 'showModuleController',
        'showMenu', 'showRouteEditingBar', 'showDataRecorder'];
    secondarySideBarOptions = ['showPOI', 'enableVoiceCommand'];

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
            option === "enableVoiceCommand"
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
