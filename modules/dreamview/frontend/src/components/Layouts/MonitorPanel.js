
import "styles/monitor.scss";

import React from "react";
import { Tab } from "react-tabs";

import CameraParam from "components/CameraParam";
import DataCollectionMonitor from "components/DataCollectionMonitor";
import PNCMonitor from "components/PNCMonitor";
import TeleOpConsole from "components/TeleopMonitor/TeleopConsole";
import { CameraVideo } from "components/Tasks/SensorCamera";
import { MONITOR_MENU } from "store/options";

export default class MonitorPanel extends React.Component {
    renderMonitor() {
        const { viewName, hmi } = this.props;

        switch (viewName) {
            case MONITOR_MENU.TELEOP_CONSOLE_MONITOR:
                return <TeleOpConsole />;
            case MONITOR_MENU.CAMERA_PARAM:
                return <CameraParam />;
            case MONITOR_MENU.DATA_COLLECTION_MONITOR:
                return <DataCollectionMonitor
                    dataCollectionUpdateStatus={hmi.dataCollectionUpdateStatus}
                    dataCollectionProgress={hmi.dataCollectionProgress}
                />;
            case MONITOR_MENU.PNC_MONITOR:
                return <PNCMonitor />;
            default:
                return null;
        }
    }

    render() {
        const { showCameraVideo } = this.props;

        return (
            <div className="right-pane">
                {showCameraVideo &&
                    <div>
                        <Tab><span>Camera View</span></Tab>
                        <CameraVideo />
                    </div>
                }
                {this.renderMonitor()}
            </div>
        );
    }
}
