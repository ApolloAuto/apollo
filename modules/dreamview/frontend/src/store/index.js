import { observable, computed, action, runInAction } from "mobx";
import * as THREE from "three";

import Meters from "store/meters";
import Monitor from "store/monitor";
import Options from "store/options";
import PARAMETERS from "store/config/parameters.yml";

class DreamviewStore {
    // Mutable States
    @observable timestamp = 0;

    @observable dimension = {
        width: window.innerWidth,
        height: window.innerHeight,
    };

    @observable meters = new Meters();

    @observable monitor = new Monitor();

    @observable options = new Options();

    @action updateTimestamp(newTimestamp) {
        this.timestamp = newTimestamp;
    }

    @action updateDimension() {
        this.dimension = {
            width: window.innerWidth,
            height: window.innerHeight,
        };
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
