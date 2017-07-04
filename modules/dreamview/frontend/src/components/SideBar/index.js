import React from "react";
import { inject, observer } from "mobx-react";

import ButtonPanel from "components/SideBar/ButtonPanel";
import Console from "components/SideBar/Console";
import Notification from "components/SideBar/Notification";

@inject("store") @observer
export default class SideBar extends React.Component {
    render() {
        const { monitor, options } = this.props.store;

        return (
            <div className="sidebar">
                <ButtonPanel showConsole={options.showConsole}
                             onConsole={() => {
                                     options.toggleShowConsole();
                                 }} />
                {options.showConsole ? <Console monitor={monitor} /> :
                 <Notification monitor={monitor} />}
            </div>
        );
    }
}
