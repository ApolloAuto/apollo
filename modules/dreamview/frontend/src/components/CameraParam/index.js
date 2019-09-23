import React, { Component } from 'react';
import ReactTooltip from 'react-tooltip';
import { inject, observer } from 'mobx-react';

import positionIcon from "assets/images/icons/position.png";
import rotationIcon from "assets/images/icons/rotation.png";

const EditSvg = ({ inEditMode }) => (
    <svg viewBox="0 0 1024 1024" width="20" height="20" fill={inEditMode ? "#FFFFFF" : "#999999"}>
        <path d="M996.3776 72.4736L951.68 27.648C934.1184 10.0608 910.4128 0 886.8096 0c-20.864 0-40.0384 7.7056-54.144 21.7856L359.168 495.7952c-0.512 0.4352-0.3584 1.1008-0.7168 1.664-0.6144 0.7936-1.2544 1.6128-1.5104 2.6368l-48.6656 178.5344c-2.8416 10.3936 0.0768 21.6064 7.7568 29.4912 5.7088 5.5808 13.312 8.6784 21.376 8.6784 2.6624 0 5.3248-0.3584 7.9616-0.9984l177.0752-48.3584c0.3072 0 0.4608 0.256 0.64 0.256a7.68 7.68 0 0 0 5.5552-2.304l473.6-473.8816c14.0288-14.08 21.7344-33.3056 21.76-54.1952 0-23.6544-10.0352-47.3344-27.6224-64.8448z m-560.128 418.6112L779.9808 146.944l97.2544 97.2288-343.8592 344.064-97.1264-97.152z m-34.2272 38.1952l92.672 92.672-127.4368 34.7904 34.7648-127.4624zM966.016 155.3152L913.408 207.9488l-97.2544-97.2288 52.6592-52.7104c5.632-5.632 12.8768-6.8096 17.9968-6.8096 10.0608 0 20.7872 4.7104 28.6208 12.5696l44.8256 44.9536c7.8336 7.8592 12.544 18.5344 12.544 28.544 0 5.1456-1.2032 12.4672-6.784 18.048z"></path>
        <path d="M921.6 947.2a25.6 25.6 0 0 1-25.6 25.6H76.8c-14.1056 0-25.6-11.4688-25.6-25.6V128c0-14.1056 11.4944-25.6 25.6-25.6h486.4V51.2H76.8a76.8 76.8 0 0 0-76.8 76.8v819.2a76.8 76.8 0 0 0 76.8 76.8h819.2a76.8 76.8 0 0 0 76.8-76.8V460.8h-51.2v486.4z"></path>
    </svg>
);

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
        this.state = {
            inEditMode: false,
        };
        this.toggleEditMode = this.toggleEditMode.bind(this);
        this.resetParam = this.resetParam.bind(this);
    }

    componentWillMount() {
        this.props.store.updateWidthInPercentage(0.8);
    }

    componentWillUnmount() {
        this.props.store.updateWidthInPercentage(1.0);
    }

    toggleEditMode() {
        this.setState({ inEditMode: !this.state.inEditMode });
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
                <div className="section-content-row" key={`${type}_${field}`}>
                    <div className="field">{field}</div>
                    <div className="value">{value}</div>
                    <div className="delta" style={{color}}>{text}</div>
                    { this.state.inEditMode &&
                        <div className="action">
                            <UpArrowSvg onClick={() => cameraData.update(
                                type, axis, adjustStep)} />
                            <DownArrowSvg onClick={() => cameraData.update(
                                type, axis, (-1) * adjustStep)} />
                        </div>
                    }
                </div>
            );
        });

        return rows;
    }

    renderCameraParam() {
        return (
            <div className="camera-param-content">
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
                <div className="camera-param-header">
                    <div className="title">Camera Parameter</div>
                    <div className="actions">
                        <div className="action" onClick={this.toggleEditMode} data-tip="Edit"
                             data-for="action">
                            <EditSvg inEditMode={this.state.inEditMode}/>
                        </div>
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
