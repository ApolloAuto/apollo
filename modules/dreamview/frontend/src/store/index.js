import { observable, computed, action, runInAction } from "mobx";
import * as THREE from "three";

import HMI from "store/hmi";
import Meters from "store/meters";
import Monitor from "store/monitor";
import Options from "store/options";
import Planning from "store/planning";
import RouteEditingManager from "store/route_editing_manager";
import Video from "store/video";
import PARAMETERS from "store/config/parameters.yml";

class DreamviewStore {
    // Mutable States
    @observable timestamp = 0;

    @observable worldTimestamp = 0;

    @observable widthInPercentage = 1.0;

    @observable dimension = {
        width: window.innerWidth,
        height: window.innerHeight,
    };

    @observable isInitialized = false;

    @observable hmi = new HMI();

    @observable planning = new Planning();

    @observable meters = new Meters();

    @observable monitor = new Monitor();

    @observable options = new Options();

    @observable video = new Video();

    @observable routeEditingManager = new RouteEditingManager();

    @observable geolocation = {};

    @action updateTimestamp(newTimestamp) {
        this.timestamp = newTimestamp;
    }

    @action updateWorldTimestamp(newTimestamp) {
        this.worldTimestamp = newTimestamp;
    }

    @action updateWidthInPercentage(newWidth) {
        this.widthInPercentage = newWidth;
        this.updateDimension();
    }

    @action updateDimension() {
        this.dimension = {
            width: window.innerWidth * this.widthInPercentage,
            height: window.innerHeight,
        };
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

    @action setPNCMonitor() {
        this.options.toggle('showPNCMonitor');
        if(this.options.showPNCMonitor) {
            this.updateWidthInPercentage(0.7);
            this.options.selectCamera('Monitor');
            this.options.showPlanningReference = true;
            this.options.showPlaningDpOptimizer = true;
            this.options.showPlanningQpOptimizer = true;
        } else {
            this.updateWidthInPercentage(1.0);
            this.options.selectCamera('Default');
            this.options.showPlanningReference = false;
            this.options.showPlaningDpOptimizer = false;
            this.options.showPlanningQpOptimizer = false;
        }
    }
}

const STORE = new DreamviewStore();

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
