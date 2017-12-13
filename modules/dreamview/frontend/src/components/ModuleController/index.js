import React from "react";
import { inject, observer } from "mobx-react";

import CheckboxItem from "components/common/CheckboxItem";
import StatusDisplay from "components/ModuleController/StatusDisplay";
import WS from "store/websocket";

@inject("store") @observer
export default class Header extends React.Component {
    render() {
        const { modes, currentMode,
                moduleStatus, hardwareStatus, displayName } = this.props.store.hmi;

        const liveModules = (currentMode !== 'none')
                              ? modes[currentMode].liveModules : Array.from(moduleStatus.keys());
        const liveHardware = (currentMode !== 'none')
                              ? modes[currentMode].liveHardware : Array.from(hardwareStatus.keys());

        const moduleEntries = liveModules.sort().map((key) => {
                  return <CheckboxItem key={key}
                                       id={key}
                                       title={displayName[key]}
                                       disabled={false}
                                       isChecked={moduleStatus.get(key)}
                                       onClick={() => {
                                            this.props.store.hmi.toggleModule(key);
                                       }}
                                       extraClasses="controller"
                                       />;
                });
        const hardwareEntries = liveHardware.map((key) => {
                  return <StatusDisplay key={key}
                                           title={displayName[key]}
                                           status={hardwareStatus.get(key)}/>;
                });

        return (
            <div className="module-controller">
                <div className="card">
                    <div className="card-header"><span>Modules</span></div>
                    <div className="card-content-row">
                        {moduleEntries}
                    </div>
                </div>
                <div className="card">
                    <div className="card-header"><span>Hardware</span></div>
                    <div className="card-content-column">
                        {hardwareEntries}
                    </div>
                </div>
            </div>
        );
    }
}