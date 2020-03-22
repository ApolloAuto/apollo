import React, { Component } from 'react';
import ReactTooltip from 'react-tooltip';
import { inject, observer } from 'mobx-react';

import positionIcon from "assets/images/icons/position.png";
import rotationIcon from "assets/images/icons/rotation.png";

const ResetSvg = () => (
    <svg viewBox="0 0 1024 1024" width="20" height="20" fill="#999999">
        <path d="M978.637 890.58H178.116V1024L0 846.551l176.115-174.78v133.42H933.94c29.353 0 44.696-12.008 44.696-41.36V496.99l88.725-141.426V800.52a88.724 88.724 0 0 1-88.725 88.058z m-88.724-667.101H133.42c-29.352 0-44.696 12.008-44.696 42.027v268.175L0 673.105v-450.96a88.724 88.724 0 0 1 88.724-88.725h800.522V0l178.116 178.116-176.115 176.115V220.81z"></path>
    </svg>
);

const UpArrowSvg = ({onClick}) => (
    <svg viewBox="0 0 1024 1024" width="15" height="15" onClick={onClick} fill="#999999">
        <path d="M820.00415442 790.08350547l-617.53286931-1e-8c-34.17530758 0-61.8167847-27.4267503-61.81678473-61.59899038 0-15.89285244 6.0951742-30.25807684 15.89285244-41.14165922l305.39062223-407.4809563c20.4634662-26.98809409 58.98852574-32.65074716 86.20054921-12.19034849 4.79147569 3.48470957 8.92343326 7.61973466 12.19034848 12.19034849L869.19806953 691.69567529c20.24260435 26.99116162 14.79774561 65.73401549-12.40814284 85.97968733C845.68548239 786.16627474 832.84481844 790.08350547 820.00415442 790.08350547L820.00415442 790.08350547z"></path>
    </svg>
);

const DownArrowSvg = ({onClick}) => (
    <svg viewBox="0 0 1024 1024" width="15" height="15" onClick={onClick} fill="#999999">
        <path d="M151.477 199.554l718.53099999 0c39.763 0 71.922 31.91 71.92200002 71.674 0 18.485-7.096 35.206-18.48600001 47.872l-355.33 474.125c-23.81 31.4-68.641 37.994-100.297 14.183-5.571-4.052-10.385-8.873-14.183-14.19l-359.398-479.178c-23.547-31.407-17.217-76.48 14.43699999-100.041 12.922-9.881 27.865-14.438 42.80500001-14.439v0l0-0.007zM151.477 199.554z"></path>
    </svg>
);

const FIELDS = {
    Position: {
        X: 'x',
        Y: 'y',
        Z: 'z'
    },
    StaticRotation: {
        Pitch: 'x',
        Yaw: 'y',
        Roll: 'z'
    },
    DynamicRotation: {
        Pitch: 'y',
        Yaw: 'z',
        Roll: 'x'
    }
};

@inject("store") @observer
export default class CameraParam extends Component {
    constructor(props) {
        super(props);

        this.resetParam = this.resetParam.bind(this);
    }

    resetParam() {
        this.props.store.cameraData.reset();
    }

    renderParamRow(type) {
        function getDeltaColorAndText(delta) {
            let color = '#FFFFFF';
            let text = '-';
            if (delta > 0) {
                color = '#1C9063';
                text = `+${delta}`;
            } else if (delta < 0) {
                color = '#B43131';
                text = delta;
            }
            return {color, text};
        };

        const { cameraData } = this.props.store;

        let step = undefined;
        switch (type) {
            case 'Position': {
                step = 2;
                break;
            };
            case 'StaticRotation':
            case 'DynamicRotation': {
                step = 3;
                break;
            }
            default:
                console.warn('Unknown parameter type:', type);
                return null;
        }

        const rows = Object.keys(FIELDS[type]).map((field) => {
            const axis = FIELDS[type][field];
            const adjustStep = Math.pow(0.1, step);
            const value = cameraData[`init${type}`].get(axis).toFixed(step);
            const delta = cameraData[`delta${type}`].get(axis).toFixed(step);
            const { color, text } = getDeltaColorAndText(delta);

            return (
                <div className="camera-param-row" key={`${type}_${field}`}>
                    <div className="field">{field}</div>
                    <div className="value">{value}</div>
                    <div className="delta" style={{color}}>{text}</div>
                    <div className="action">
                        <UpArrowSvg onClick={() => cameraData.update(
                            type, axis, adjustStep)} />
                        <DownArrowSvg onClick={() => cameraData.update(
                            type, axis, (-1) * adjustStep)} />
                    </div>
                </div>
            );
        });

        return rows;
    }

    renderCameraParam() {
        return (
            <div className="monitor-content">
                <div className="section">
                    <div className="section-title"
                         data-tip="Camera position in world coordinate system"
                         data-for="param">
                        <img height="20px" width="20px" src={positionIcon} />
                        <span>Position</span>
                    </div>
                    <div className="section-content">
                        {this.renderParamRow('Position')}
                    </div>
                </div>
                <div className="section">
                    <div className="section-title">
                        <img height="18px" width="20px" src={rotationIcon} />
                        <span>Rotation</span>
                    </div>
                    <div className="section-subtitle"
                         data-tip="Camera rotation in IMU coordinate system, default facing to Z negative direction"
                         data-for="param">
                        Static
                    </div>
                    <div className="section-content">
                        {this.renderParamRow('StaticRotation')}
                    </div>
                    <div className="section-subtitle"
                         data-tip="IMU rotation in world coordinate system"
                         data-for="param">
                        Dynamic
                    </div>
                    <div className="section-content">
                        {this.renderParamRow('DynamicRotation')}
                    </div>
                </div>
                <ReactTooltip id="param" place="right" delayShow={300} multiline />
            </div>
        );
    }

    render() {
        return (
            <div className="monitor camera-param">
                <div className="monitor-header">
                    <div className="title">Camera Parameter</div>
                    <div className="actions">
                        <div className="action" onClick={this.resetParam} data-tip="Reset"
                             data-for="action">
                            <ResetSvg />
                        </div>
                        <ReactTooltip id="action" place="left" delayShow={100} />
                    </div>
                </div>
                {this.renderCameraParam()}
            </div>
        );
    }
}
