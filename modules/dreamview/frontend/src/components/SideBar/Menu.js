import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";

import RadioItem from 'components/common/RadioItem';

import menuData from 'store/config/MenuData';
import perceptionIcon from "assets/images/menu/perception.png";
import predictionIcon from "assets/images/menu/prediction.png";
import routingIcon from "assets/images/menu/routing.png";
import decisionIcon from "assets/images/menu/decision.png";
import planningIcon from "assets/images/menu/planning.png";
import cameraIcon from "assets/images/menu/point_of_view.png";
import positionIcon from "assets/images/menu/position.png";
import mapIcon from "assets/images/menu/map.png";

import { POINT_CLOUD_WS } from "store/websocket";

const MenuIconMapping = {
        perception: perceptionIcon,
        prediction: predictionIcon,
        routing: routingIcon,
        decision: decisionIcon,
        planning: planningIcon,
        camera: cameraIcon,
        position: positionIcon,
        map: mapIcon,
};

const MenuIdOptionMapping = {
        perceptionPointCloud: 'showPointCloud',
        perceptionVehicle: 'showObstaclesVehicle',
        perceptionPedestrian: 'showObstaclesPedestrian',
        perceptionBicycle: 'showObstaclesBicycle',
        perceptionUnknownMovable: 'showObstaclesUnknownMovable',
        perceptionUnknownUnmovable: 'showObstaclesUnknownUnmovable',
        perceptionUnknown: 'showObstaclesUnknown',
        perceptionVirtual: 'showObstaclesVirtual',
        perceptionCipv: 'showObstaclesCipv',
        perceptionVelocity: 'showObstaclesVelocity',
        perceptionHeading: 'showObstaclesHeading',
        perceptionId: 'showObstaclesId',
        perceptionLaneMarker: 'showPerceptionLaneMarker',
        predictionMajor: 'showPredictionMajor',
        predictionMinor: 'showPredictionMinor',
        routing: 'showRouting',
        decisionMain: 'showDecisionMain',
        decisionObstacle: 'showDecisionObstacle',
        planningCar: 'showPlanningCar',
        planningReference: 'showPlanningReference',
        planningDpOptimizer: 'showPlanningDpOptimizer',
        planningQpOptimizer: 'showPlanningQpOptimizer',
        planningLine: 'showPlanning',
        positionLocalization: 'showPositionLocalization',
        positionGps: 'showPositionGps',
        mapCrosswalk: 'showMapCrosswalk',
        mapClearArea: 'showMapClearArea',
        mapJunction: 'showMapJunction',
        mapLane: 'showMapLane',
        mapRoad: 'showMapRoad',
        mapSignal: 'showMapSignal',
        mapStopSign: 'showMapStopSign',
};

@observer
class MenuItemCheckbox extends React.Component {
    render() {
        const {id, title, options} = this.props;
        return (
            <ul>
                <li id={id} onClick={() => {
                    options.toggle(MenuIdOptionMapping[id]);
                    if (id === "perceptionPointCloud") {
                        POINT_CLOUD_WS.togglePointCloud(options.showPointCloud);
                    }
                }}>
                    <div className="switch">
                        <input type="checkbox" name={id} className="toggle-switch"
                        id={id} checked={options[MenuIdOptionMapping[id]]} readOnly/>
                        <label className="toggle-switch-label" htmlFor={id} />
                    </div>
                    <span>{title}</span>
                </li>
            </ul>
        );
    }
}

@observer
class SubMenu extends React.Component {
    render() {
        const {tabId, tabTitle, tabType, data, options} = this.props;
        let entries = null;
        if (tabType === 'checkbox') {
            entries = Object.keys(data)
                .map(key => {
                    const item = data[key];
                    if (options.hideOptionToggle[key]) {
                        return null;
                    }
                    return (
                        <MenuItemCheckbox key={key} id={key} title={item}
                        options={options}/>
                    );
                });
        } else if (tabType === 'radio') {
            entries = Object.keys(data)
                .map(key => {
                    const item = data[key];
                    if (options.hideOptionToggle[key]) {
                        return null;
                    }
                    return (
                        <RadioItem key={`${tabId}_${key}`} id={tabId}
                                   onClick={() => {
                                            options.selectCamera(item);
                                   }}
                                   checked={options.cameraAngle === item}
                                   title={item} options={options}/>
                    );
                });
        }
        const result = (
            <div className="card">
                <div className="card-header summary">
                    <span>
                        <img src={MenuIconMapping[tabId]}/>
                        {tabTitle}
                    </span>
                </div>
                <div className="card-content-column">{entries}</div>
            </div>
        );
        return result;
    }
}

@observer
export default class Menu extends React.Component {
    render() {
        const { options } = this.props;

        const subMenu = Object.keys(menuData)
            .map(key => {
                const item = menuData[key];

                if (OFFLINE_PLAYBACK && !item.supportInOfflineView) {
                    return null;
                } else {
                    return (
                        <SubMenu key={item.id} tabId={item.id} tabTitle={item.title}
                                 tabType={item.type} data={item.data} options={options} />
                    );
                }
            });

        return (
            <div className="tool-view-menu" id="layer-menu">
                {subMenu}
            </div>
        );
    }
}
