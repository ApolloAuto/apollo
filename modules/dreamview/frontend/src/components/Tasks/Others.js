import React from "react";
import { inject, observer } from "mobx-react";

import CheckboxItem from "components/common/CheckboxItem";
import WS from "store/websocket";

@inject("store") @observer
export default class Others extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly } = this.props.store;

        const disablePanel = enableHMIButtonsOnly || options.tasksPanelLocked;

        return (
            <div className="others card">
                <div className="card-header"><span>Others</span></div>
                <div className="card-content-column">
                    <button disabled={disablePanel}
                            onClick={() => {
                                WS.resetBackend();
                            }}>Reset Backend Data</button>
                    <button disabled={disablePanel}
                            onClick={() => {
                                WS.dumpMessages();
                            }}>Dump Message</button>
                    <CheckboxItem id={"showPNCMonitor"}
                                  title={"PNC Monitor"}
                                  isChecked={options.showPNCMonitor}
                                  disabled={disablePanel}
                                  onClick={() => {
                                      this.props.store.handleOptionToggle('showPNCMonitor');
                                  }}/>
                    <CheckboxItem id={"toggleSimControl"}
                                  title={"SimControl"}
                                  isChecked={options.simControlEnabled}
                                  disabled={options.tasksPanelLocked}
                                  onClick={() => {
                                      WS.toggleSimControl(!options.simControlEnabled);
                                      this.props.store.handleOptionToggle('simControlEnabled');
                                  }}/>
                    <CheckboxItem id={"showVideo"}
                                  title={"Camera Sensor"}
                                  isChecked={options.showVideo}
                                  disabled={disablePanel}
                                  onClick={() => {
                                      this.props.store.handleOptionToggle('showVideo');
                                  }}/>
                    <CheckboxItem id={"panelLock"}
                                  title={"Lock Task Panel"}
                                  isChecked={options.tasksPanelLocked}
                                  disabled={false}
                                  onClick={() => {
                                    this.props.store.handleOptionToggle('tasksPanelLocked');
                                  }}/>
                </div>
            </div>
        );
    }
}