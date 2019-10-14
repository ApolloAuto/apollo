import { observable, action } from "mobx";

export default class TeleopStore {
    @observable audioEnabled = false;
    @observable micEnabled = false;
    @observable videoEnabled = false;

    @action toggleAudio() {
        this.audioEnabled = !this.audioEnabled;
    }

    @action toggleMic() {
        this.micEnabled = !this.micEnabled;
    }

    @action toggleVideo() {
        this.videoEnabled = !this.videoEnabled;
    }

    @action update(status) {
        if (typeof status.audio === "boolean") {
            this.audioEnabled = status.audio;
        }
        if (typeof status.mic === "boolean") {
            this.micEnabled = status.mic;
        }
        if (typeof status.video === "boolean") {
            this.videoEnabled = status.video;
        }
    }
}
