import { observable, action, computed } from "mobx";

import PARAMETERS from "store/config/parameters.yml";

export default class Options {
    // Side Bar options
    @observable showModuleController = PARAMETERS.options.defaults.showModuleController;
    @observable showPNCMonitor = PARAMETERS.options.defaults.showPNCMonitor;
    @observable showRouteEditingBar = PARAMETERS.options.defaults.showRouteEditingBar;
    @observable showPOI = PARAMETERS.options.defaults.showPOI;
    @observable showVideo = PARAMETERS.options.defaults.showVideo;

    @observable showMenu =
        OFFLINE_PLAYBACK ? true : PARAMETERS.options.defaults.showMenu;
    @observable showTasks =
        OFFLINE_PLAYBACK ? false : PARAMETERS.options.defaults.showTasks;

    mutuallyExclusiveOptions = ['showTasks', 'showModuleController',
        'showMenu', 'showRouteEditingBar'];

    // Layer Menu options
    @observable showDecisionMain = PARAMETERS.options.defaults.showDecisionMain;
    @observable showDecisionObstacle = PARAMETERS.options.defaults.showDecisionObstacle;
    @observable showPlanning = PARAMETERS.options.defaults.showPlanning;
    @observable showPlanningReference = PARAMETERS.options.defaults.showPlanningReference;
    @observable showPlaningDpOptimizer = PARAMETERS.options.defaults.showPlaningDpOptimizer;
    @observable showPlanningQpOptimizer = PARAMETERS.options.defaults.showPlanningQpOptimizer;
    @observable showRouting = PARAMETERS.options.defaults.showRouting;
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
    @observable showObstaclesVelocity =
        PARAMETERS.options.defaults.showObstaclesVelocity;
    @observable showObstaclesHeading =
        PARAMETERS.options.defaults.showObstaclesHeading;
    @observable showObstaclesId =
        PARAMETERS.options.defaults.showObstaclesId;
    @observable cameraAngle = PARAMETERS.options.defaults.cameraAngle;


    @computed get showTools() {
        return this.showTasks ||
               this.showModuleController ||
               this.showMenu ||
               this.showPOI;
    }

    @computed get showGeo() {
        return this.showRouteEditingBar ||
               this.cameraAngle === 'Map' ||
               this.cameraAngle === 'Overhead' ||
               this.cameraAngle === 'Monitor';
    }

    @action toggleSideBar(option) {
        this[option] = !this[option];

        // Disable other mutually exclusive options
        if (this[option] && this.mutuallyExclusiveOptions.includes(option)) {
            for (const other of this.mutuallyExclusiveOptions) {
                if (other !== option) {
                    this[other] = false;
                }
            }
        }
    }

    @action toggle(option) {
        this[option] = !this[option];
    }

    @action selectCamera(option) {
        this.cameraAngle = option;
    }
}
