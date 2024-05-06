import React from 'react';
import { inject, observer } from 'mobx-react';

import SplitPane from 'react-split-pane';
import Header from 'components/Header';
import MainView from 'components/Layouts/MainView';
import ToolView from 'components/Layouts/ToolView';
import MonitorPanel from 'components/Layouts/MonitorPanel';
import SideBar from 'components/SideBar';

import ApplicationGuideModal from 'components/ApplicationGuideModal';

import HOTKEYS_CONFIG from 'store/config/hotkeys.yml';
import WS, { MAP_WS, POINT_CLOUD_WS, CAMERA_WS } from 'store/websocket';

@inject('store') @observer
export default class Dreamview extends React.Component {
  constructor(props) {
    super(props);
    this.handleDrag = this.handleDrag.bind(this);
    this.handleKeyPress = this.handleKeyPress.bind(this);
    this.updateDimension = this.props.store.dimension.update.bind(this.props.store.dimension);
  }

  handleDrag(masterViewWidth) {
    const { options, dimension } = this.props.store;
    if (options.showMonitor) {
      dimension.updateMonitorWidth(
        Math.min(
          Math.max(window.innerWidth - masterViewWidth, 0),
          window.innerWidth,
        ),
      );
    }
  }

  handleKeyPress(event) {
    const { options, enableHMIButtonsOnly, hmi } = this.props.store;

    const optionName = HOTKEYS_CONFIG[event.key];
    if (!optionName || options.showDataRecorder
      || options.showDefaultRoutingInput || options.showCycleNumberInput
      || options.showFuelClient) {
      return;
    }

    event.preventDefault();
    if (optionName === 'cameraAngle') {
      // press 'v' to switch camera angle
      options.rotateCameraAngle();
    } else if (
      !options.isSideBarButtonDisabled(optionName, enableHMIButtonsOnly, hmi.inNavigationMode)
    ) {
      this.props.store.handleOptionToggle(optionName);
    }
  }

  componentWillMount() {
    this.props.store.dimension.initialize();
  }

  componentDidMount() {
    WS.initialize();
    MAP_WS.initialize();
    POINT_CLOUD_WS.initialize();
    CAMERA_WS.initialize();
    window.addEventListener('resize', this.updateDimension, false);
    window.addEventListener('keypress', this.handleKeyPress, false);
  }

  componentWillUnmount() {
    window.removeEventListener('resize', this.updateDimension, false);
    window.removeEventListener('keypress', this.handleKeyPress, false);
  }

  render() {
    const { dimension, options, hmi } = this.props.store;
    const { currentVehicleType } = hmi;

    return (
            <div>
                <Header />
                <div className="pane-container">
                    <SplitPane
                        split="vertical"
                        size={dimension.pane.width}
                        onChange={this.handleDrag}
                        allowResize={options.showMonitor}
                    >
                        <div className="left-pane">
                            <SideBar />
                            <div className="dreamview-body">
                                <MainView />
                                <ToolView />
                            </div>
                        </div>
                        <MonitorPanel
                            hmi={hmi}
                            viewName={options.monitorName}
                            showVideo={options.showVideo}
                        />
                    </SplitPane>
                </div>
              {
                // When the current vehicle is dkit series, the safety pop-up is displayed
                (currentVehicleType > 0 && currentVehicleType <= 7) &&
                <ApplicationGuideModal />}
            </div>
    );
  }
}
