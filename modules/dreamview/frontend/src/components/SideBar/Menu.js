import _ from 'lodash';
import { inject, observer } from 'mobx-react';
import React from 'react';

import RadioItem from 'components/common/RadioItem';

import decisionIcon from 'assets/images/menu/decision.png';
import mapIcon from 'assets/images/menu/map.png';
import perceptionIcon from 'assets/images/menu/perception.png';
import planningIcon from 'assets/images/menu/planning.png';
import cameraIcon from 'assets/images/menu/point_of_view.png';
import positionIcon from 'assets/images/menu/position.png';
import predictionIcon from 'assets/images/menu/prediction.png';
import routingIcon from 'assets/images/menu/routing.png';
import menuData from 'store/config/MenuData';

import { POINT_CLOUD_WS } from 'store/websocket';

import './style.scss';

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

@inject('store') @observer
class MenuItemCheckbox extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      channels: [],
    };
  }

  componentDidMount() {
    const {
      id,
    } = this.props;
    if (id === 'perceptionPointCloud') {
      POINT_CLOUD_WS.getPointCloudChannel().then((channels) => {
        this.setState({ channels });
      }).catch(
        err => {
          this.setState({ channels: [] });
        }
      );
    }
  }

  onStatusSelectChange = (event) => {
    if (event.target.value) {
      POINT_CLOUD_WS.changePointCloudChannel(event.target.value);
    }
  };

  render() {
    const {
      id,
      title,
      optionName,
      options,
      isCustomized,
      store,
    } = this.props;

    const { hmi } = store;
    return (
      <ul className="item">
        <li
          id={id}
          onClick={() => {
            options.toggle(optionName, isCustomized);
            if (id === 'perceptionPointCloud') {
              POINT_CLOUD_WS.togglePointCloud(options.showPointCloud);
            }
          }}
        >
          <div className="switch">
            <input
              type="checkbox"
              name={id}
              className="toggle-switch"
              id={id}
              checked={isCustomized ? options.customizedToggles.get(optionName)
                : options[optionName]}
              readOnly
            />
            <label className="toggle-switch-label" htmlFor={id} />
          </div>
          <span>{title}</span>
          {id === 'perceptionPointCloud' && <span className='point_cloud_channel_select'>
            <span className="arrow" />
            <select
              onClick={(e) => e.stopPropagation()}
              value={hmi.currentPointCloudChannel}
              onChange={this.onStatusSelectChange}
            >
              <option key={'select-channel'} value={''}>- select channel -</option>
              {
                this.state.channels.map((channel) => {
                  return (
                    <option key={channel} value={channel}>{channel}</option>
                  );
                })
              }
            </select>
          </span>}
        </li>
      </ul>
    );
  }
}

@observer
class SubMenu extends React.Component {
  constructor(props) {
    super(props);

    this.menuIdOptionMapping = {};
    for (const name in PARAMETERS.options) {
      const option = PARAMETERS.options[name];
      if (option.menuId) {
        this.menuIdOptionMapping[option.menuId] = name;
      }
    }
  }

  render() {
    const {
      tabId, tabTitle, tabType, data, options,
    } = this.props;
    let entries = null;
    if (tabType === 'checkbox') {
      entries = Object.keys(data)
        .map((key) => {
          const item = data[key];
          if (options.togglesToHide[key]) {
            return null;
          }
          return (
            <MenuItemCheckbox
              key={key}
              id={key}
              title={item}
              optionName={this.menuIdOptionMapping[key]}
              options={options}
              isCustomized={false}
            />
          );
        });
      if (tabId === 'planning' && options.customizedToggles.size > 0) {
        const extraEntries = options.customizedToggles.keys().map((pathName) => {
          const title = _.startCase(_.snakeCase(pathName));
          return (
            <MenuItemCheckbox
              key={pathName}
              id={pathName}
              title={title}
              optionName={pathName}
              options={options}
              isCustomized
            />
          );
        });
        entries = entries.concat(extraEntries);
      }
    } else if (tabType === 'radio') {
      // Now we only have camera tab using radio in menu
      if (tabId === 'camera') {
        const cameraAngles = Object.values(data)
          .filter((angle) => PARAMETERS.options.cameraAngle[`has${angle}`] !== false);
        entries = cameraAngles.map((item) => (
          <RadioItem
            key={`${tabId}_${item}`}
            id={tabId}
            onClick={() => {
              options.selectCamera(item);
            }}
            checked={options.cameraAngle === item}
            title={_.startCase(item)}
          />
        ));
      }
    }
    const result = (
      <div className="card">
        <div className="card-header summary">
          <span>
            <img src={MenuIconMapping[tabId]} />
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
export default class LayerMenu extends React.Component {
  render() {
    const { options } = this.props;

    const subMenu = Object.keys(menuData)
      .map((key) => {
        const item = menuData[key];

        if (OFFLINE_PLAYBACK && !item.supportInOfflineView) {
          return null;
        }
        return (
          <SubMenu
            key={item.id}
            tabId={item.id}
            tabTitle={item.title}
            tabType={item.type}
            data={item.data}
            options={options}
          />
        );
      });

    return (
      <div className="tool-view-menu" id="layer-menu">
        {subMenu}
      </div>
    );
  }
}
