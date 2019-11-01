import { observable, computed, action } from "mobx";

import PLAYBACK_STYLE from "styles/playback-controls.scss";
import MAIN_STYLE from "styles/main.scss";

export const MAP_SIZE = {
    DEFAULT: 'default',
    FULL: 'full',
};

export default class Dimension {
    // left pane to right pane ratio
    @observable paneWidthRatio = 1;

    // the left pane that excludes monitor and header.
    @observable pane = {
        width: window.innerWidth,
        height: window.innerHeight,
    };

    // the main view inside the left pane that excludes side bar and tool view
    @observable main = {
        width: window.innerWidth,
        height: window.innerHeight,
    };

    // the 3D scene
    @observable scene = {
        width: window.innerWidth,
        height: window.innerHeight,
    };

    // the navigation map
    @observable navigation = {
        width: window.innerWidth,
        height: window.innerHeight,
        size: MAP_SIZE.DEFAULT,
    };

    constructor(hmi, options) {
        this.hmi = hmi;
        this.options = options;
    }

    isSmallScreen() {
        return window.innerHeight < 800.0 || window.innerWidth < 1280.0;
    }

    @action updateWidthInPercentage(newRatio) {
        this.paneWidthRatio = newRatio;
        this.update();
    }

    @action enableMonitor() {
        this.updateWidthInPercentage(0.7);
    }

    @action disableMonitor() {
        this.updateWidthInPercentage(1.0);
    }

    @computed get shouldDivideSceneAndMapSpace() {
        return this.hmi.inTeleopMode && this.navigation.size === MAP_SIZE.FULL;
    }

    @action updatePaneDimension() {
        let offset = 0;
        if (!OFFLINE_PLAYBACK) {
            offset = this.isSmallScreen()
                ? MAIN_STYLE.HEADER_HEIGHT_SMALL : MAIN_STYLE.HEADER_HEIGHT_LARGE;
        }

        this.pane.width = window.innerWidth * this.paneWidthRatio;
        this.pane.height = window.innerHeight - offset;
    }

    @action updateMainDimension() {
        let heightRatio = 1;
        if (this.options.showTools) {
            heightRatio = OFFLINE_PLAYBACK ? 0.65 : 0.60;
        }

        let widthOffset = 0;
        if (!OFFLINE_PLAYBACK) {
            widthOffset = this.isSmallScreen()
                ? MAIN_STYLE.SIDE_BAR_WIDTH_SMALL : MAIN_STYLE.SIDE_BAR_WIDTH_LARGE;
        }

        this.main.width = Math.max(MAIN_STYLE.MIN_MAIN_VIEW_WIDTH, this.pane.width - widthOffset);
        this.main.height = this.pane.height * heightRatio;
    }

    @action updateSceneDimension() {
        const parent = this.main;

        let width = parent.width;
        let height = parent.height;
        if (OFFLINE_PLAYBACK) {
            height = parent.height - PLAYBACK_STYLE.PLAYBACK_CONTROL_HEIGHT;
        } else if (this.options.showCameraView) {
            // Align the aspect ratio of 3D scene's to camera image's
            // for visual comparison between the two layers.
            height = parent.width / this.cameraData.imageAspectRatio;
        } else if (this.shouldDivideSceneAndMapSpace) {
            width = parent.width / 2;
        }

        this.scene.width = width;
        this.scene.height = height;
    }

    @action updateNavigationDimension() {
        const mainWidth = this.main.width;
        const sceneHeight = this.scene.height;
        switch (this.navigation.size) {
            case MAP_SIZE.FULL:
                this.navigation.height = sceneHeight;
                this.navigation.width = this.hmi.inTeleopMode ? mainWidth / 2 : mainWidth;
                break;
            case MAP_SIZE.DEFAULT:
                this.navigation.height = Math.min(sceneHeight * 0.5, 300);
                this.navigation.width = Math.min(mainWidth * 0.3, 250);
                break;
        }
    }

    @action toggleNavigationSize() {
        const oldSize = this.navigation.size;
        this.navigation.size =
            (oldSize === MAP_SIZE.FULL) ? MAP_SIZE.DEFAULT : MAP_SIZE.FULL;

        this.updateSceneDimension();
        this.updateNavigationDimension();
    }

    initialize() {
        if (this.options.showMonitor) {
            this.enableMonitor();
        }
        this.update();
    }

    update() {
        this.updatePaneDimension();
        this.updateMainDimension();
        this.updateSceneDimension();
        this.updateNavigationDimension();
    }
}
