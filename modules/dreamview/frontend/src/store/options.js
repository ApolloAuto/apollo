import { observable, action } from "mobx";

import PARAMETERS from "store/config/parameters.yml";

export default class Options {
    @observable showPOI = PARAMETERS.options.defaults.showPOI;
    @observable showMenu = PARAMETERS.options.defaults.showMenu;
    @observable showConsole = PARAMETERS.options.defaults.showConsole;
    @observable showPNCMonitor = PARAMETERS.options.defaults.showPNCMonitor;
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

    @action toggleShowPOI() {
        this.showPOI = !this.showPOI;
        if (this.showPOI) {
            this.showMenu = false;
            this.showConsole = false;
        }
    }
    @action toggleShowMenu() {
        this.showMenu = !this.showMenu;
        if (this.showMenu) {
            this.showPOI = false;
            this.showConsole = false;
        }
    }
    @action toggleShowConsole() {
        this.showConsole = !this.showConsole;
        if (this.showConsole) {
            this.showPOI = false;
            this.showMenu = false;
        }
    }
    @action toggle(option) {
        this[option] = !this[option];
    }
    @action selectCamera(option) {
        this.cameraAngle = option;
    }
}
