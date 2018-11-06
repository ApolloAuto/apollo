import React from "react";

export class CameraVideo extends React.Component {
    render() {
        return (
            <div className="camera-video">
                <img src='/image'/>
            </div>
        );
    }
}

export default class SensorCamera extends React.Component {
    render() {
        return (
            <div className="card" style={{minWidth: '265px'}}>
                <div className="card-header"><span>Camera Sensor</span></div>
                <div className="card-content-column">
                    <CameraVideo />
                </div>
            </div>
        );
    }
}
