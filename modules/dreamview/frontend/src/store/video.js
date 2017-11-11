import { observable, computed, action, runInAction } from "mobx";

import JDataView from 'jDataView';

export default class Video {

    @observable path = '';

    @observable startTime = 0; // in epoch sec

    file = null;

    offset = null;

    @action setPath(newPath) {
        this.path = newPath;
    }

    @action setStartTime(newStartTime) {
        this.startTime = newStartTime;
    }

    @action resetVideo() {
        this.path = '';
        this.startTime = 0;
        this.file = null;
        this.offset = null;
    }

    @computed get showVideo() {
        return (this.path !== undefined && this.path.length !== 0);
    }

    static VALID_TYPE() {
        return ['video/mp4', 'video/quicktime'];
    }

    isValidType(type) {
        return type && Video.VALID_TYPE().includes(type);
    }

    readBlock(onLoadHandler) {
        const reader = new FileReader();
        const ATOM_HEADER_SIZE = 8;
        const blob = this.file.slice(this.offset, this.offset + ATOM_HEADER_SIZE);
        reader.onload = onLoadHandler;
        reader.readAsArrayBuffer(blob);
    }

    findMoovAtom(event) {
        if (event.target.error) {
            console.error("ERROR:", event.target.error);
            return;
        }

        const atom_header = event.target.result;
        const data = new JDataView(new Uint8Array(atom_header));
        const atom_type = data.getString(4,4);
        const atom_size = data.getUint32(0);

        const found = (atom_type === 'moov');
        this.offset = found
            ? this.offset + event.target.result.byteLength
            : this.offset + atom_size;
        const nextOnLoadHandler = found
            ? this.findMovieHeaderAtom.bind(this)
            : this.findMoovAtom.bind(this);

        if(this.offset <= this.file.size) {
            this.readBlock(nextOnLoadHandler);
        } else {
            console.warn("Cannot find video creation time.");
        }
    }

    findMovieHeaderAtom(event) {
        if (event.target.error) {
            console.error("ERROR:", event.target.error);
            return;
        }

        const atom_header = event.target.result;
        const data = new JDataView(new Uint8Array(atom_header));
        const atom_type = data.getString(4,4);
        const atom_size = data.getUint32(0);

        if (atom_type === 'cmov') {
            console.warn("Cannot find video creation time: moov atom is compressed.");
            return;
        } else if (atom_type !== 'mvhd') {
            console.warn("Cannot find video creation time: expected header 'mvhd' not found.");
            return;
        } else {
            this.offset = this.offset + event.target.result.byteLength + 4;
            if(this.offset <= this.file.size) {
                this.readBlock(this.findCreationTime.bind(this));
            } else {
               console.warn("Cannot find video creation time.");
            }
        }
    }

    findCreationTime(event) {
        if (event.target.error) {
            console.error("ERROR:", event.target.error);
            return;
        }

        // difference between Unix epoch and QuickTime epoch, in seconds
        const EPOCH_ADJUSTER = 2082844800;
        const atom_header = event.target.result;
        const data = new JDataView(new Uint8Array(atom_header));
        const atom_size = data.getUint32(0);
        const creationTimeInEpoch = atom_size - EPOCH_ADJUSTER;
        this.setStartTime(creationTimeInEpoch);
    }

    extractCreationTime(file) {
        this.file = file;
        this.offset = 0;
        this.readBlock(this.findMoovAtom.bind(this));
    }

    setVideo(file) {
        const URL = window.URL || window.webkitURL;
        const fileURL = URL.createObjectURL(file);
        if (this.isValidType(file.type)) {
            this.setPath(fileURL);
            this.extractCreationTime(file);
        } else {
            alert('Invalid file found. ' +
                'Please select mp4 video only.');
        }
    }
}
