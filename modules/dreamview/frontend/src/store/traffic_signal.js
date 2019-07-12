import { observable, action } from "mobx";
import _ from 'lodash';

export default class TrafficSignal {
    @observable color = "";

    @action update(world) {
        this.color = _.get(world, 'trafficSignal.currentSignal');
    }
}
