import { observable, action } from "mobx";

export default class TrafficSignal {
    @observable color = "";

    @action update(world) {
        if (world.trafficSignal !== undefined) {
            this.color = world.trafficSignal.currentSignal;
        }
    }
}
