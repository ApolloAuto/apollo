import React from "react";
import { inject, observer } from "mobx-react";
import Slider from 'react-rangeslider';

import ControlIcons from "components/PlaybackControls/ControlIcons";
import WS from "store/websocket";

@inject("store") @observer
export default class PlaybackControls extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            rate: 1.0,
            isPlaying: false,
            nextAction: 'play',
        };

        this.handleRateChange = this.handleRateChange.bind(this);
        this.handleIconClick = this.handleIconClick.bind(this);
        this.handleSliderChange = this.handleSliderChange.bind(this);
        this.handleSliderChangeComplete = this.handleSliderChangeComplete.bind(this);
    }

    handleRateChange(event) {
        const { playback } = this.props.store;

        const newRate = parseFloat(event.target.value);
        this.setState({rate: newRate});
        playback.setPlayRate(newRate);

        if (this.state.isPlaying) {
            WS.startPlayback(playback.msPerFrame);
        }
    }

    handleIconClick(event) {
        const { playback } = this.props.store;

        const isPlaying = !this.state.isPlaying;
        this.setState({isPlaying: isPlaying});

        switch (this.state.nextAction) {
            case 'play':
                WS.startPlayback(playback.msPerFrame);
                break;
            case 'pause':
                WS.pausePlayback();
                break;
            case 'replay':
                playback.resetCurrentFrame();
                WS.startPlayback(playback.msPerFrame);
                break;
        }
    }

    handleSliderChange(value) {
        const { playback } = this.props.store;
        playback.updateCurrentFrameByPercentage(value);
    }

    handleSliderChangeComplete() {
        if (!this.state.isPlaying) {
            const { playback } = this.props.store;
            WS.requestSimulationWorld(playback.jobId, playback.currentFrame);
        }
    }

    componentWillUpdate(nextProps, nextState) {
        const { percentage, replayComplete,
                currentTimeSec, totalTimeSec } = this.props.store.playback;

        if (nextProps.store.playback.replayComplete && nextState.isPlaying) {
            this.setState({isPlaying: false});
        }


        let newAction = null;
        if (replayComplete) {
            newAction = 'replay';
        } else if (nextState.isPlaying) {
            newAction = 'pause';
        } else {
            newAction = 'play';
        }
        if (nextState.nextAction !== newAction) {
            this.setState({nextAction: newAction});
        }
    }

    render() {
        const { percentage, replayComplete,
                currentTimeSec, totalTimeSec } = this.props.store.playback;

        return (
            <div className="playback-controls">
                <ControlIcons type={this.state.nextAction} onClick={this.handleIconClick}/>
                <div className="rate-selector">
                    <select onChange={this.handleRateChange} value={this.state.rate}>
                        <option value={0.25}>x 0.25</option>
                        <option value={0.5}>x 0.5</option>
                        <option value={1.0}>x 1.0</option>
                        <option value={2.0}>x 2.0</option>
                    </select>
                    <span className="arrow"></span>
                </div>
                <div className="scrubber">
                    <Slider tooltip={false} value={percentage}
                            onChange={this.handleSliderChange}
                            onChangeComplete={this.handleSliderChangeComplete}/>
                </div>
                <div className="time-display">{`${currentTimeSec} / ${totalTimeSec} s`}</div>
            </div>
        );
    }
}