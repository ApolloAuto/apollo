import { observable, computed, action } from 'mobx';

import PLAYBACK_STYLE from 'styles/playback-controls.scss';
import MAIN_STYLE from 'styles/main.scss';
import { MONITOR_MENU } from 'store/options';

export const MAP_SIZE = {
  DEFAULT: 'default',
  FULL: 'full',
};

const MONITOR_WIDTH_IN_PX = {
  [MONITOR_MENU.PNC_MONITOR]: {
    small: 400,
    large: 450,
  },
  [MONITOR_MENU.DATA_COLLECTION_MONITOR]: {
    small: 500,
    large: 640,
  },
  [MONITOR_MENU.CONSOLE_TELEOP_MONITOR]: {
    small: 240,
    large: 270,
  },
  [MONITOR_MENU.CAR_TELEOP_MONITOR]: {
    small: 240,
    large: 270,
  },
  [MONITOR_MENU.CAMERA_PARAM]: {
    small: 400,
    large: 450,
  },
  [MONITOR_MENU.FUEL_CLIENT]: {
    small: 400,
    large: 450,
  },
  default: {
    small: 200,
    large: 240,
  },
};

const MAX_TOOL_VIEW_HEIGHT_IN_PX = {
  small: 333,
  large: 380,
};

export default class Dimension {
    // width of the right pane
    @observable monitorWidth = 0;

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

    @action updateMonitorWidth(width) {
      this.monitorWidth = width;
      this.update();
    }

    @action enableMonitor() {
      const width = MONITOR_WIDTH_IN_PX[this.options.monitorName] || MONITOR_WIDTH_IN_PX.default;
      this.updateMonitorWidth(this.isSmallScreen() ? width.small : width.large);
    }

    @action disableMonitor() {
      this.updateMonitorWidth(0.0);
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

      this.pane.width = window.innerWidth - this.monitorWidth;
      this.pane.height = window.innerHeight - offset;
    }

    @action updateMainDimension() {
      this.main.height = this.pane.height;
      if (this.options.showTools) {
        const heightRatio = OFFLINE_PLAYBACK ? 0.65 : 0.60;
        const maxHeight = this.isSmallScreen()
          ? MAX_TOOL_VIEW_HEIGHT_IN_PX.small : MAX_TOOL_VIEW_HEIGHT_IN_PX.large;
        this.main.height = Math.max(
          this.pane.height - maxHeight,
          this.pane.height * heightRatio);
      }

      let widthOffset = 0;
      if (!OFFLINE_PLAYBACK) {
        widthOffset = this.isSmallScreen()
          ? MAIN_STYLE.SIDE_BAR_WIDTH_SMALL : MAIN_STYLE.SIDE_BAR_WIDTH_LARGE;
      }
      this.main.width = Math.max(MAIN_STYLE.MIN_MAIN_VIEW_WIDTH, this.pane.width - widthOffset);
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
          this.navigation.width = this.shouldDivideSceneAndMapSpace
            ? mainWidth / 2 : mainWidth;
          break;
        case MAP_SIZE.DEFAULT:
          this.navigation.height = Math.min(sceneHeight * 0.5, 300);
          this.navigation.width = Math.min(mainWidth * 0.3, 250);
          break;
      }
    }

    @action toggleNavigationSize() {
      const oldSize = this.navigation.size;
      this.navigation.size = (oldSize === MAP_SIZE.FULL) ? MAP_SIZE.DEFAULT : MAP_SIZE.FULL;

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
