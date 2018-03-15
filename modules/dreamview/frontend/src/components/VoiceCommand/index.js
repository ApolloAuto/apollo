import React from 'react';
import { inject, observer } from "mobx-react";
import Recorder from "recorder-js";
import Wav from "node-wav";

import STORE from "store";
import WS from "store/websocket";

// Utils.
function Downsample(buffer, in_sample_rate, out_sample_rate) {
    const Average = arr => arr.reduce( ( p, c ) => p + c, 0 ) / arr.length;
    if (in_sample_rate === out_sample_rate) {
        return buffer;
    } else if (in_sample_rate < out_sample_rate) {
        throw "Cannot increase sample rate.";
    }

    const sample_ratio = in_sample_rate / out_sample_rate;
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
export default class VoiceCommand extends React.Component {
    constructor(props) {
        super(props);
        this.handleStream = this.handleStream.bind(this);
        this.handleError = this.handleError.bind(this);
        this.handleStart = this.handleStart.bind(this);
        this.handleStop = this.handleStop.bind(this);
        // TODO(xiaoxq): Get updated from HMIConfig.
        this.audio_capturing_conf = {
            channels: 1,
            sample_rate: 16000,
            bit_depth: 16,
        };
    }

    componentWillMount() {
        if (this.recorder) {
            this.handleStart();
        } else {
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
                        buffer[0],
                        this.audio_context.sampleRate,
                        this.audio_capturing_conf.sample_rate);
                    const wav = Wav.encode([downsampled], {
                        sampleRate: this.audio_capturing_conf.sample_rate,
                        bitsPerSample: this.audio_capturing_conf.bits_per_sample,
                    });
                    const WAV_HEADER_LEN = 44;
                    WS.sendVoicePiece(wav.slice(WAV_HEADER_LEN));
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

