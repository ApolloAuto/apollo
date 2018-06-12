import { observable, computed, action, autorun } from "mobx";

import HMI from "store/hmi";
import ControlData from "store/control_data";
import Meters from "store/meters";
import Monitor from "store/monitor";
import Options from "store/options";
import PlanningData from "store/planning_data";
import Playback from "store/playback";
import RouteEditingManager from "store/route_editing_manager";
import TrafficSignal from "store/traffic_signal";
import PARAMETERS from "store/config/parameters.yml";

class DreamviewStore {
    // Mutable States
    @observable timestamp = 0;

    @observable worldTimestamp = 0;

    @observable sceneDimension = {
        width: window.innerWidth,
        height: window.innerHeight,
        widthRatio: 1,
    }

    @observable dimension = {
        width: window.innerWidth,
        height: window.innerHeight,
    };

    @observable isInitialized = false;

    @observable hmi = new HMI();

    @observable planningData = new PlanningData();

    @observable controlData = new ControlData();

    @observable playback = OFFLINE_PLAYBACK ? new Playback() : null;

    @observable trafficSignal = new TrafficSignal();

    @observable meters = new Meters();

    @observable monitor = new Monitor();

    @observable options = new Options();

    @observable routeEditingManager = new RouteEditingManager();

    @observable geolocation = {};

    @observable moduleDelay = observable.map();

    @computed get enableHMIButtonsOnly() {
        return !this.isInitialized;
    }

    @action updateTimestamp(newTimestamp) {
        this.timestamp = newTimestamp;
    }

    @action updateWidthInPercentage(newRatio) {
        this.sceneDimension.widthRatio = newRatio;
    }

    @action setInitializationStatus(status){
        this.isInitialized = status;
    }

    @action updatePlanning(newPlanningData) {
        this.planning.update(newPlanningData);
    }

    @action setGeolocation(newGeolocation) {
        this.geolocation = newGeolocation;
    }

    @action enablePNCMonitor() {
        this.updateWidthInPercentage(0.7);
        this.options.showPlanningReference = true;
        this.options.showPlanningDpOptimizer = true;
        this.options.showPlanningQpOptimizer = true;
    }

    @action disablePNCMonitor() {
        this.updateWidthInPercentage(1.0);
        this.options.showPlanningCar = false;
        this.options.showPlanningReference = false;
        this.options.showPlanningDpOptimizer = false;
        this.options.showPlanningQpOptimizer = false;
    }

    @action updateModuleDelay(world) {
        if(world && world.delay) {
            for(module in world.delay) {
                const hasNotUpdated = (world.delay[module] < 0);
                const delay = hasNotUpdated ? '-' : world.delay[module].toFixed(2);
                if (this.moduleDelay.has(module)){
                    this.moduleDelay.get(module).delay = delay;
                } else {
                    this.moduleDelay.set(module, {
                        delay: delay,
                        name: module[0].toUpperCase() + module.slice(1),
                    });
                }
            }
        }
    }

    handleOptionToggle(option) {
        const oldShowPNCMonitor = this.options.showPNCMonitor;
        const oldShowRouteEditingBar = this.options.showRouteEditingBar;

        this.options.toggle(option);

        // disable tools turned off after toggling
        if (oldShowPNCMonitor && !this.options.showPNCMonitor) {
            this.disablePNCMonitor();
        }
        if (oldShowRouteEditingBar && !this.options.showRouteEditingBar) {
            this.routeEditingManager.disableRouteEditing();
        }

        // enable selected tool
        if (this.options[option]) {
            switch(option) {
                case "showPNCMonitor":
                    this.enablePNCMonitor();
                    break;
                case 'showRouteEditingBar':
                    this.options.showPOI = false;
                    this.routeEditingManager.enableRouteEditing();
                    break;
            }
        }
    }

    setOptionStatus(option, enabled) {
        this.options[option] = (enabled || false);
    }

    // This function is triggerred automatically whenever a observable changes
    updateDimension() {
        let offsetX = 0;
        let offsetY = 0;
        let mainViewHeightRatio = 0.65;
        if (!OFFLINE_PLAYBACK) {
            const smallScreen = window.innerHeight < 800.0;
            offsetX = smallScreen ? 80 : 90; // width of side-bar
            offsetY = smallScreen ? 55 : 60; // height of header
            mainViewHeightRatio = 0.60;
        }

        this.dimension.width = window.innerWidth * this.sceneDimension.widthRatio;
        this.dimension.height = window.innerHeight - offsetY;

        this.sceneDimension.width = this.dimension.width - offsetX;
        this.sceneDimension.height = this.options.showTools
                ? this.dimension.height * mainViewHeightRatio : this.dimension.height;
    }
}

const STORE = new DreamviewStore();

autorun(() => {
    STORE.updateDimension();
});

// For debugging purpose only. When turned on, it will insert a random
// monitor message into monitor every 10 seconds.
const timer = PARAMETERS.debug.autoMonitorMessage ? setInterval(() => {
    const item = [
        {
            level: "FATAL",
            message: "There is a fatal hardware issue detected. It might be " +
                     "due to an incorrect power management setup. Please " +
                     "see the logs for details."
        }, {
            level: "WARN",
            message: "The warning indicator on the instrument panel is on. " +
                     "This is usually due to a failure in engine."
        }, {
            level: "ERROR",
            message: "Invalid coordinates received from the " +
                     "localization module.",
        }, {
            level: "INFO",
            message: "Monitor module has started and is succesfully " +
                     "initialized.",
        }][Math.floor(Math.random() * 4)];
    STORE.monitor.insert(item.level, item.message, Date.now());
}, 10000) : null;

export default STORE;
