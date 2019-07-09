import { observable, action } from "mobx";

export default class TrafficSignal {
    @observable color = "";

    @action update(world) {
        if (world.planningSignal !== undefined) {
            this.color = world.planningSignal;
        }
    }
}
