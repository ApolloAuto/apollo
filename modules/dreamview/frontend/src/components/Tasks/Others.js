import React from "react";
import { inject, observer } from "mobx-react";

import CheckboxItem from "components/common/CheckboxItem";
import WS from "store/websocket";

@inject("store") @observer
export default class Others extends React.Component {
    render() {
        const { options, enableHMIButtonsOnly } = this.props.store;
        const { isPanelLocked, toggleLock } = this.props;

        const disablePanel = enableHMIButtonsOnly || isPanelLocked;

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
                    <CheckboxItem id={"showVideo"}
                                  title={"Camera Sensor"}
                                  isChecked={options.showVideo}
                                  disabled={disablePanel}
                                  onClick={() => {
                                        options.toggleSideBar("showVideo");
                                  }}/>
                    <CheckboxItem id={"showPNCMonitor"}
                                  title={"PNC Monitor"}
                                  isChecked={options.showPNCMonitor}
                                  disabled={disablePanel}
                                  onClick={() => {
                                        this.props.store.handleSideBarClick('showPNCMonitor');
                                  }}/>
                    <CheckboxItem id={"panelLock"}
                                  title={"Lock Task Panel"}
                                  isChecked={isPanelLocked}
                                  disabled={false}
                                  onClick={toggleLock}/>
                </div>
            </div>
        );
    }
}