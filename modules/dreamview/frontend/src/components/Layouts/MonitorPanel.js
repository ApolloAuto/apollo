
import "styles/monitor.scss";

import React from "react";
import { Tabs, TabList, Tab, TabPanel } from "react-tabs";

import { MONITOR_MENU } from "store/options";

import CameraParam from "components/CameraParam";
import { CameraVideo } from "components/Tasks/SensorCamera";
import DataCollectionMonitor from "components/DataCollectionMonitor";
import PNCMonitor from "components/PNCMonitor";
import ConsoleTeleOp from "components/TeleopMonitor/ConsoleTeleop";
import CarTeleOp from "components/TeleopMonitor/CarTeleop";

export default class MonitorPanel extends React.Component {
    renderMonitor() {
        const { viewName, hmi } = this.props;

        switch (viewName) {
            case MONITOR_MENU.CONSOLE_TELEOP_MONITOR:
                return <ConsoleTeleOp />;
            case MONITOR_MENU.CAR_TELEOP_MONITOR:
                return <CarTeleOp />;
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
                    <Tabs>
                        <TabList>
                            <Tab>Camera View</Tab>
                        </TabList>
                        <TabPanel>
                            <CameraVideo />
                        </TabPanel>
                    </Tabs>
                }
                {this.renderMonitor()}
            </div>
        );
    }
}
