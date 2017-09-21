import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

// import DashCamButton from "components/SideBar/DashCamButton";

@observer
class SideBarButton extends React.Component {
    render() {
        const { onClick, active, label, extraClasses } = this.props;
        return (
            <button onClick={onClick}
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
        const { onClick, video } = this.props;

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
                    className="button">
                DashCam Video
            </button>
          </div>
        );
    }
}

@observer
export default class ButtonPanel extends React.Component {
    openHMI() {
        const server = window.location.origin;
        const link = document.createElement("a");
        link.href = server;
        window.open(
            `http://${link.hostname}:8887`, "_self");
    }

    render() {
        const { showRouteEditingBar,
                sendDefaultRoutingRequest,
                onConsole, showConsole,
                onMenu, showMenu,
                onVideo } = this.props;

        return (
            <div>
                <SideBarButton label="HMI Setup" active={false}
                               onClick={this.openHMI.bind(this)}
                               extraClasses={["button-corner"]} />
                <div className="separator" />
                <SideBarButton label="Default Routing"
                               onClick={sendDefaultRoutingRequest}
                               active={false} />
                <div className="separator" />
                <SideBarButton label="Route Editing"
                               onClick={showRouteEditingBar}
                               active={false} />
                <div className="separator" />
                <DashCamButton onClick={onVideo}/>
                <div className="separator" />
                <SideBarButton label="Notifications"
                               onClick={onConsole}
                               active={showConsole} />
                <div className="separator" />
                <SideBarButton label="Layer Menu"
                               onClick={onMenu}
                               active={showMenu} />
            </div>
        );
    }
}
