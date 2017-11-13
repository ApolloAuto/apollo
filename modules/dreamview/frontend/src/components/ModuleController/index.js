import React from "react";
import { inject, observer } from "mobx-react";

import Controller from "components/ModuleController/Controller";
import StatusDisplay from "components/ModuleController/StatusDisplay";
import WS from "store/websocket";

@inject("store") @observer
export default class Header extends React.Component {
    render() {
        const { modules, hardware, displayName } = this.props.store.hmi;

        const moduleEntries = Array.from(modules.keys()).sort().map((key) => {
                  return <Controller key={key}
                                           id={key}
                                           title={displayName[key]}
                                           modules={modules}
                                           onClick={() => {
                                                this.props.store.hmi.toggleModule(key);
                                           }}/>;
                });
        const hardwareEntries = Array.from(hardware.keys()).sort().map((key) => {
                  return <StatusDisplay key={key}
                                           title={displayName[key]}
                                           status={hardware.get(key)}/>;
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