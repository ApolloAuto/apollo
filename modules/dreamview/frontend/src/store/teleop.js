import { observable, action } from "mobx";

export default class TeleopStore {
    @observable audioEnabled = false;
    @observable micEnabled = false;
    @observable videoEnabled = false;
    @observable modemInfo = observable.map();

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

        for (const name in status.modems) {
            this.modemInfo.set(name, status.modems[name]);
        }
    }
}
