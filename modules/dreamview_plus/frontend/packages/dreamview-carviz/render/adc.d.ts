export default class Adc {
    adc: any;

    shadowAdc: any;

    planningAdc: any;

    isInitialized: boolean;

    private scene;

    private adcVisible;

    private planningAdcVisible;

    private shadowAdcVisible;

    toggleAdcVisible(bool: any): void;

    togglePlanningAdcVisible(bool: any): void;

    toggleShadowAdcVisible(bool: any): void;

    constructor(scene: any);

    private init;

    update(pos: any, name: any): void;
}
