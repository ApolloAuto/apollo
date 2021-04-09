import { action, observable } from "mobx";


const TIME_RANGE_IN_SEC = 300;
export default class Latency {
    @observable lastUpdatedTime = 0;
    data = {};

    @action
    updateTime(newTime) {
        this.lastUpdatedTime = newTime;
    }

    updateLatencyGraph(moduleName, newLatency) {
        if (!newLatency) {
            return;
        }

        const currentTime = newLatency.timestampSec;
        let graph = this.data[moduleName];
        if (graph.length > 0) {
            const startTimeSec = graph[0].x;
            const endTimeSec = graph[graph.length - 1].x;
            const diff = currentTime - startTimeSec;
            if (currentTime < endTimeSec) {
                // new data set, clean up existing one
                this.data[moduleName] = [];
                graph = this.data[moduleName];
            } else if (diff > TIME_RANGE_IN_SEC) {
                // shift out old data
                graph.shift();
            }
        }

        if (graph.length === 0 || graph[graph.length - 1].x !== currentTime) {
            graph.push({ x: currentTime, y: newLatency.totalTimeMs });
        }
    }

    update(world) {
        if (world.latency) {
            let timestamp = 0;
            for (const moduleName in world.latency) {
                if (!(moduleName in this.data)) {
                    this.data[moduleName] = [];
                }
                this.updateLatencyGraph(moduleName, world.latency[moduleName]);
                timestamp = Math.max(world.latency[moduleName].timestampSec, timestamp);
            }
            this.updateTime(timestamp);
        }
    }
}
