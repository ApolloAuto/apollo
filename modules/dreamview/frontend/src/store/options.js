import { observable, action } from "mobx";

import PARAMETERS from "store/config/parameters.yml";

export default class Options {
    @observable showConsole = PARAMETERS.options.defaults.showConsole;
    @observable showPlanning = PARAMETERS.options.defaults.showPlanning;

    // TODO: Consider having a per-type config about whether to show obstacles
    @observable showObstacles = PARAMETERS.options.defaults.showObstacles;
    @observable showDecision = PARAMETERS.options.defaults.showDecision;
    @observable showPredictionMajor = PARAMETERS.options.defaults.showPredictionMajor;
    @observable showPredictionMinor = PARAMETERS.options.defaults.showPredictionMinor;

    @action toggleShowConsole() {
        this.showConsole = !this.showConsole;
    }
}
