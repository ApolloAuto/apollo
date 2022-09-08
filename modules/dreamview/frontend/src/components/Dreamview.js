import React from 'react';
import { inject, observer } from 'mobx-react';

import SplitPane from 'react-split-pane';
import Header from 'components/Header';
import MainView from 'components/Layouts/MainView';
import ToolView from 'components/Layouts/ToolView';
import MonitorPanel from 'components/Layouts/MonitorPanel';
import SideBar from 'components/SideBar';

import HOTKEYS_CONFIG from 'store/config/hotkeys.yml';
import WS, { MAP_WS, POINT_CLOUD_WS, CAMERA_WS, ACCOUNT_WS, CONFIGURATION_WS } from 'store/websocket';
import ApplicationGuideModal from './ApplicationGuideModal';

@inject('store') @observer
export default class Dreamview extends React.Component {
  constructor(props) {
    super(props);
    this.handleDrag = this.handleDrag.bind(this);
    this.handleFocus = this.handleFocus.bind(this);
    this.handleBlur = this.handleBlur.bind(this);
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

  /**
   * handle focus event
   *
   * check and change mode by elemment type
   *
   * @param {Event} event event object
   */
  handleFocus(event) {
    // if input element actived, enter insert mode
    const tagName = event.target.tagName && event.target.tagName.toUpperCase();
    const disabled = event.target.disabled;
    const type = event.target.type;
    if (!disabled && tagName === 'TEXTAREA') {
      this.props.store.setInsertMode();
    } else if (!disabled && tagName === 'INPUT' && type === 'text') {
      this.props.store.setInsertMode();
    }

  }

  /**
   * handle blur event
   *
   * check and change mode by elemment type
   *
   * @param {Event} event event object
   */
  handleBlur(event) {
    // if input element blur, enter normal mode
    const tagName = event.target.tagName && event.target.tagName.toUpperCase();
    const disabled = event.target.disabled;
    const type = event.target.type;
    if (!disabled && tagName === 'TEXTAREA') {
      this.props.store.setNormalMode();
    } else if (!disabled && tagName === 'INPUT' && type === 'text') {
      this.props.store.setNormalMode();
    }
  }

  handleKeyPress(event) {
    const { options, enableHMIButtonsOnly, hmi, mode } = this.props.store;

    if (mode !== 'normal') {
      // TODO: support other modes
      return;
    }
    // valid only in normal mode
    // TODO: add more features and support visual mode
    const optionName = HOTKEYS_CONFIG[event.key];
    if (!optionName || options.showDataRecorder
      || options.showDefaultRoutingInput || options.showCycleNumberInput
      || options.showFuelClient || options.showParkTimeInput
    ) {
      return;
    }

    event.preventDefault();
    if (optionName === 'cameraAngle') {
      options.rotateCameraAngle();
    } else if (!options.isSideBarButtonDisabled(
      optionName, enableHMIButtonsOnly, hmi.inNavigationMode)) {
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
    ACCOUNT_WS.initialize();
    CONFIGURATION_WS.initialize();
    window.addEventListener('resize', this.updateDimension, false);
    window.addEventListener('keypress', this.handleKeyPress, false);
  }

  componentWillUnmount() {
    window.removeEventListener('resize', this.updateDimension, false);
    window.removeEventListener('keypress', this.handleKeyPress, false);
  }

  render() {
    const { dimension, options, hmi } = this.props.store;

    return (
      <div>
        <Header />
        <div className="pane-container"
          onFocus={this.handleFocus}
          onBlur={this.handleBlur}
        >
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
        <ApplicationGuideModal />
      </div>
    );
  }
}
