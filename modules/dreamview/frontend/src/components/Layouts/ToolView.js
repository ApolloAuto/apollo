import React from "react";
import { inject, observer } from "mobx-react";

import ModuleController from "components/ModuleController";
import Console from "components/SideBar/Console";
import Menu from "components/SideBar/Menu";
import POI from "components/SideBar/POI";
import QuickStarter from "components/QuickStarter";

@inject("store") @observer
export default class ToolView extends React.Component {
    render() {
        const { monitor, options, routeEditingManager } = this.props.store;

        return (
            <div className="tools">
                {options.showModuleController && <ModuleController />}
                {options.showQuickStarter && <QuickStarter />}
                {options.showMenu && <Menu options={options} /> }
                {options.showPOI && <POI routeEditingManager={routeEditingManager}
                                         options={options}/>}
                {options.showConsole && <Console monitor={monitor} />}
            </div>
        );
    }
}