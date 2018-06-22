import React from "react";
import { inject, observer } from "mobx-react";

import SplitPane from 'react-split-pane';
import Header from "components/Header";
import MainView from "components/Layouts/MainView";
import ToolView from "components/Layouts/ToolView";
import PNCMonitor from "components/PNCMonitor";
import SideBar from "components/SideBar";
import VoiceCommand from "components/VoiceCommand";
import WS, {MAP_WS, POINT_CLOUD_WS} from "store/websocket";


@inject("store") @observer
export default class Dreamview extends React.Component {
    constructor(props) {
        super(props);
        this.handleDrag = this.handleDrag.bind(this);
    }

    handleDrag(masterViewWidth) {
        const { options } = this.props.store;
        if (options.showPNCMonitor) {
            this.props.store.updateWidthInPercentage(
                Math.min(1.00, masterViewWidth / window.innerWidth));
        }
    }

    componentWillMount() {
        this.props.store.updateDimension();
    }

    componentDidMount() {
        WS.initialize();
        MAP_WS.initialize();
        POINT_CLOUD_WS.initialize();
        window.addEventListener("resize", () => {
            this.props.store.updateDimension();
        });
    }

    render() {
        const { isInitialized, dimension, sceneDimension, options, hmi } = this.props.store;

        return (
            <div>
                <Header />
                <div className="pane-container">
                    <SplitPane split="vertical"
                           size={dimension.width}
                           onChange={this.handleDrag}
                           allowResize={options.showPNCMonitor}>
                        <div className="left-pane">
                            <SideBar />
                            <div className="dreamview-body">
                                <MainView />
                                <ToolView />
                            </div>
                        </div>
                        <div className="right-pane">
                            {options.showPNCMonitor && <PNCMonitor />}
                        </div>
                    </SplitPane>
                </div>
                <div className="hidden">
                    {options.enableVoiceCommand && <VoiceCommand />}
                </div>
            </div>
        );
    }
}