import { observable, action } from "mobx";

import PARAMETERS from "store/config/parameters.yml";

export default class Options {
    @observable showConsole = PARAMETERS.options.defaults.showConsole;
    @observable showPlanning = PARAMETERS.options.defaults.showPlanning;

    @action toggleShowConsole() {
        this.showConsole = !this.showConsole;
    }
}
