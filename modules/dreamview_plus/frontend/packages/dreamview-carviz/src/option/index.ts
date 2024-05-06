import { layerVisible } from './layerVisible';

export default class Option {
    public layerOption;

    private key;

    constructor() {
        this.key = null;
        this.setLayerOption();
    }

    setLayerOption = () => {
        let result = null;
        if (this.key) {
            const option = localStorage.getItem(`layerOption_${this.key}`);
            result = JSON.parse(option);
        } else {
            result = layerVisible;
        }
        this.layerOption = result;
    };

    updateLayerOption = (option, key) => {
        this.key = key;
        localStorage.setItem(`layerOption_${this.key}`, JSON.stringify(option));
        this.setLayerOption();
    };
}
