import React from "react";
import { inject, observer } from "mobx-react";

import ControlIcons from "components/PlaybackControls/ControlIcons";
import TimeControls from "components/PlaybackControls/TimeControls";
import WS from "store/websocket";

const PLAY_BACK_SPEED = [0.5, 1.0, 1.5, 2.0, 3.0];

@inject("store") @observer
export default class PlaybackControls extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            rate: 1.0,
            isPlaying: false,
            nextScreenMode: 'normalscreen',
        };

        this.nextAction = 'play';

        this.handleRateChange = this.handleRateChange.bind(this);
        this.handleFrameSeek = this.handleFrameSeek.bind(this);
        this.handleActionChange = this.handleActionChange.bind(this);
        this.handleScreenModeChange = this.handleScreenModeChange.bind(this);
    }

    handleRateChange(event) {
        const { playback } = this.props.store;

        const newRate = parseFloat(event.target.value);
        this.setState({ rate: newRate });
        playback.setPlayRate(newRate);

        if (this.state.isPlaying) {
            WS.startPlayback(playback.msPerFrame);
        }
    }

    handleActionChange() {
        const { playback } = this.props.store;

        const isPlaying = !this.state.isPlaying;
        playback.setPlayAction(isPlaying);
        this.setState({ isPlaying: isPlaying });

        switch (this.nextAction) {
            case 'play':
                WS.startPlayback(playback.msPerFrame);
                break;
            case 'pause':
                WS.pausePlayback();
                break;
            case 'replay':
                playback.resetFrame();
                WS.startPlayback(playback.msPerFrame);
                break;
        }
    }

    handleScreenModeChange() {
        switch (this.state.nextScreenMode) {
            case 'fullscreen':
                this.props.store.setOptionStatus('showMenu', false);
                this.setState({ nextScreenMode: 'normalscreen' });
                break;
            case 'normalscreen':
                this.props.store.setOptionStatus('showMenu', true);
                this.setState({ nextScreenMode: 'fullscreen' });
                break;
        }
    }

    handleFrameSeek(frame) {
        const { playback } = this.props.store;

        playback.seekFrame(frame);
        if (!this.state.isPlaying) {
            WS.requestSimulationWorld(playback.recordId, frame);
        } else if (!WS.requestTimer) {
            WS.startPlayback(playback.msPerFrame);
        }
    }

    componentWillUpdate(nextProps, nextState) {
        const { playback } = this.props.store;

        if (playback.replayComplete && this.state.isPlaying) {
            playback.setPlayAction(false);
            this.setState({ isPlaying: false });
        }

        if (playback.replayComplete && !playback.isSeeking) {
            this.nextAction = 'replay';
        } else if (nextState.isPlaying) {
            this.nextAction = 'pause';
        } else {
            this.nextAction = 'play';
        }
    }

    render() {
        const { playback } = this.props.store;

        return (
            <div className="playback-controls">
                <ControlIcons extraClasses="left-controls"
                              onClick={this.handleActionChange}
                              type={this.nextAction} />
                <div className="rate-selector">
                    <select onChange={this.handleRateChange} value={this.state.rate}>
                        {PLAY_BACK_SPEED.map(
                            (speed) => (
                                <option key={speed} value={speed}>
                                    {`x ${speed.toFixed(1)}`}
                                </option>
                            ))
                        }
                    </select>
                    <span className="arrow"></span>
                </div>
                <TimeControls numFrames={playback.numFrames}
                              loadingMarker={playback.loadingMarker}
                              currentFrame={playback.currentFrame}
                              fps={playback.FPS}
                              isSeeking={playback.isSeeking}
                              handleFrameSeek={this.handleFrameSeek} />
                <ControlIcons extraClasses="right-controls"
                              onClick={this.handleScreenModeChange}
                              type={this.state.nextScreenMode} />
            </div>
        );
    }
}