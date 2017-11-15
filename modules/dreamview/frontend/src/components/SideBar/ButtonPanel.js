import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";


class SideBarButton extends React.Component {
    render() {
        const { disabled, onClick, active, label, extraClasses } = this.props;
        return (
            <button onClick={onClick}
                    disabled={disabled}
                    className={classNames({
                            "button": true,
                            "active": active,
                        }, extraClasses)}>
                {label}
            </button>
        );
    }
}

class DashCamButton extends React.Component {
    constructor(props) {
      super(props);
      this.onClickHandler = this.onClickHandler.bind(this);
    }

    onClickHandler() {
      this.fileInput.value = null;
      this.fileInput.click();
    }

    render() {
        const { disabled, onClick, video } = this.props;

        return (
          <div>
            <input  style={{display: "none"}}
                    ref={(input) => {
                        this.fileInput = input;
                    }}
                    type="file"
                    accept="video/*"
                    onChange={onClick}/>
            <button onClick={this.onClickHandler}
                    disabled={disabled}
                    className="button">
                DashCam Video
            </button>
          </div>
        );
    }
}


export default class ButtonPanel extends React.Component {
    render() {
        const { initialized,
                onQuickStarter, showQuickStarter,
                onModuleController, showModuleController,
                onMenu, showMenu,
                onRouteEditingBar, showRouteEditingBar,
                onPOI, showPOI,
                onPNCMonitor, showPNCMonitor,
                onConsole, showConsole,
                resetBackend, dumpMessages, onVideo} = this.props;

        return (
            <div>
                <SideBarButton label="Quick Start"
                               disabled={false}
                               onClick={onQuickStarter}
                               active={showQuickStarter}/>
                <SideBarButton label="Module Controller"
                               disabled={false}
                               onClick={onModuleController}
                               active={showModuleController}/>
                <SideBarButton label="Layer Menu"
                               disabled={!initialized}
                               onClick={onMenu}
                               active={showMenu} />
                <SideBarButton label="Route Editing"
                               disabled={!initialized}
                               onClick={onRouteEditingBar}
                               active={showRouteEditingBar} />
                <SideBarButton label="Default Routing"
                               disabled={!initialized}
                               onClick={onPOI}
                               active={showPOI} />
                <SideBarButton label="PNC Monitor"
                               disabled={!initialized}
                               onClick={onPNCMonitor}
                               active={showPNCMonitor} />
                <SideBarButton label="Notifications"
                               disabled={!initialized}
                               onClick={onConsole}
                               active={showConsole} />
                <SideBarButton label="Reset Backend Data"
                               disabled={!initialized}
                               onClick={resetBackend}
                               active={false} />
                <SideBarButton label="Dump Messages"
                               disabled={!initialized}
                               onClick={dumpMessages}
                               active={false} />
                <DashCamButton disabled={!initialized}
                               onClick={onVideo}/>
            </div>
        );
    }
}
