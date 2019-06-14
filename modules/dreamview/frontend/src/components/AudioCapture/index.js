import React from 'react';
import { inject, observer } from "mobx-react";
import Recorder from "recorder-js";

import WS from "store/websocket";

// This component will record and send audio to backend in pieces of every
// 100ms, which could be used as voice command or oral log.
//
// Please note that Chrome has strict restriction on microphone control. To use
// the audio capture facility, you should use browsers like Firefox and grant
// requried permissions.

// Utils.
function PCM16Encode(channel) {
    const kBitDepth = 16;

    const samples = channel.length;
    const buffer = new ArrayBuffer(samples * (kBitDepth >> 3));
    const output = new Int16Array(buffer, 0);
    let pos = 0;
    for (let i = 0; i < samples; ++i) {
        let v = Math.max(-1, Math.min(channel[i], 1));
        v = ((v < 0) ? v * 32768 : v * 32767) | 0;
        output[pos++] = v;
    }
    return Buffer(buffer);
}

function Downsample(buffer, in_sample_rate) {
    // TODO(xiaoxq): Get updated from HMIConfig.
    const kOutSampleRate = 16000;

    const Average = arr => arr.reduce( ( p, c ) => p + c, 0 ) / arr.length;
    if (in_sample_rate === kOutSampleRate) {
        return buffer;
    } else if (in_sample_rate < kOutSampleRate) {
        throw "Cannot increase sample rate.";
    }

    const sample_ratio = in_sample_rate / kOutSampleRate;
    const new_len = Math.round(buffer.length / sample_ratio);
    const result = new Float32Array(new_len);

    for (let i = 0; i < new_len; ++i) {
        result[i] = Average(buffer.slice(
            Math.ceil(i * sample_ratio), Math.ceil((i + 1) * sample_ratio)));
    }
    return result;
}

// Public component.
@inject("store") @observer
export default class AudioCapture extends React.Component {
    constructor(props) {
        super(props);
        this.handleStream = this.handleStream.bind(this);
        this.handleError = this.handleError.bind(this);
        this.handleStart = this.handleStart.bind(this);
        this.handleStop = this.handleStop.bind(this);
    }

    componentWillMount() {
        if (this.recorder) {
            this.handleStart();
        } else {
            if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia) {
                alert('Browser does not support using media device.');
                this.props.store.handleOptionToggle('enableAudioCapture');
                return;
            }
            navigator.mediaDevices.getUserMedia({audio: true})
                .then(this.handleStream)
                .catch(this.handleError);
        }
    }

    componentWillUnmount() {
        this.handleStop();
    }

    handleError(err) {
        console.log(err);
    }

    // Record and send data piece every 100ms.
    handleStart() {
        // Start the first cycle.
        this.recorder.start();

        this.timer = setInterval(() => {
            this.recorder.stop().then(
                ({blob, buffer}) => {
                    const downsampled = Downsample(
                        buffer[0], this.audio_context.sampleRate);
                    WS.sendAudioPiece(PCM16Encode(downsampled));
                    // Start next cycle.
                    this.recorder.start();
                }
            );
        }, 100);
    }

    handleStop() {
        clearInterval(this.timer);
        if (this.recorder) {
            this.recorder.stop();
        }
    }

    handleStream(media_stream) {
        // Init recorder.
        this.audio_context = new (window.AudioContext || window.webkitAudioContext)();
        this.recorder = new Recorder(this.audio_context);
        this.recorder.init(media_stream);
        this.handleStart();
    }

    render() {
        // No rendering.
        return (null);
    }
}
