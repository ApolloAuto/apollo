import { toJS } from 'mobx';
import { inject, observer } from 'mobx-react';
import React from 'react';

import { Empty, Radio } from 'antd';

import { throttle } from 'lodash';
import WS, { PLUGIN_WS } from 'store/websocket';
import LocalDynamicModelsItem from './LocalDynamicModelsItem';
import LocalRecordItem from './LocalRecordItem';
import LocalScenarioSetItem from './LocalScenarioSetItem';
import { ScenarioCertificateInvalid, ScenarioNoCertificate }
  from './ScenarioNoCertificate';
import RemoteResourseItem from './RemoteResourseItem';
import VehicleListItem from './VehicleListItem';
import V2xList from './V2xList';

const RadioGroup = Radio.Group;

@inject('store')
@observer
export default class DataProfile extends React.Component {

  constructor(props) {
    super(props);
    PLUGIN_WS.initialize();
    this.state = {
      currentKey: 'scenarioProfiles',
      tabs: [
        {
          title: 'Scenario Profiles',
          key: 'scenarioProfiles',
        },
        {
          title: 'Dynamic Model',
          key: 'dynamicModel',
        },
        {
          title: 'Record Profiles',
          key: 'recordProfiles',
        },
        {
          title: 'Vehicle Profiles',
          key: 'vehicleProfiles',
        },
        {
          title: 'v2x Profiles',
          key: 'v2xProfiles',
        },
      ],
    };
  }

  componentDidMount() {
    const { store } = this.props;
    setTimeout(() => {
      // 校验ws是否连接，确认链接后再校验证书
      PLUGIN_WS.checkWsConnection()
        .checkCertificate();
      WS.checkWsConnection().loadLoocalScenarioSets();
      WS.checkWsConnection().loadLocalRecords();
      const { enableSimControl } = store.options;
      if (enableSimControl) {
        WS.getDymaticModelList();
      }
      PLUGIN_WS.getVehicleInfo();
    }, 300);
  }

  onTypeSelectChange = (e) => {
    const { store } = this.props;
    store.studioConnector.updateTypeConditionValue(e.target.value);
  };

  onStatusSelectChange = (e) => {
    const { store } = this.props;
    store.studioConnector.updateStatusConditionValue(e.target.value);
  };

  onRadioChange = (e) => {
    WS.changeScenarioSet(e.target.value);
  };

  onDynamicModelChange = (e) => {
    WS.changeDynamicModel(e.target.value);
  };

  onRecordChange = (item) => {
    WS.changeRecord(item);
  };

  // 渲染动力学模型tab
  renderDynamicModelList = () => {
    const { store } = this.props;
    const { currentDynamicModel, dynamicModels } = store.hmi;
    const { enableSimControl } = store.options;
    if (!enableSimControl) {
      return <div>Please open SimControl to switch the dynamic model</div>;
    }
    return (<div className='local-scenario-set-list'>
      <RadioGroup
        onChange={this.onDynamicModelChange}
        value={currentDynamicModel}
      >
        {toJS(dynamicModels).map((item) => {
          return (
            <LocalDynamicModelsItem
              key={item}
              item={item}
              currentDynamicModel={currentDynamicModel}
            />
          );
        })
        }
      </RadioGroup>
    </div>);
  };

  // 更新record列表节流函数
  updateRecordList = throttle(() => {
    WS.checkWsConnection().loadLocalRecords();
  }, 5000);

  // 渲染数据包tab
  renderRecordProfilesList = () => {
    const { store } = this.props;
    /**
     * @param currentRecordId string
     * @param records {id: number}
     */
    const { currentRecordId, records } = store.hmi;
    const { enableSimControl } = store.options;
    if (enableSimControl) {
      return <div>Please close SimControl to switch the records player.</div>;
    }
    return (<div className='local-record-list'>
      {Object.keys(toJS(records)).map((item) => {
        // record下载状态
        const recordStatus = records[item];
        return (
          <LocalRecordItem
            key={item}
            item={item}
            updateRecordList={this.updateRecordList}
            // 0 下载中 1 下载完成
            recordStatus={recordStatus}
            currentRecordId={currentRecordId}
            changeRecord={this.onRecordChange}
          />
        );
      })
      }
    </div>);
  };

  // 渲染车辆注册
  renderVehicleProfilesList = () => {
    const { store } = this.props;
    const {
      vehicleInfoList,
      vehicleUpdateStatus,
    } = store.studioConnector;
    return (<div className='remote-vehicle-list'>
      {/*更新后 没有车辆*/}
      {(vehicleInfoList.length === 0 && vehicleUpdateStatus !== 0) &&
        <div className='remote-vehicle-list_empty'>
        <Empty
          style={{color: '#fff'}}
          image={Empty.PRESENTED_IMAGE_SIMPLE}
          description="There is no vehicle under your account."/>
        </div>}
      {/*更新后 存在车辆*/}
        {
          (vehicleInfoList.length > 0 && vehicleUpdateStatus !== 0) &&
          vehicleInfoList.map((item) =>
            <div className='remote-vehicle-list-item-container'>
              <VehicleListItem value={item.vehicle_id} item={item}/>
            </div>
          )
        }
    </div>);
  };

  // 渲染v2x tab
  renderV2xProfilesList = () => <V2xList />;

  render() {

    const { store } = this.props;

    const {
      certificateStatus,
      remoteScenarioSetList,
      remoteDynamicModelList,
      remoteRecordList,
      scenarioSet,
      currentScenarioSetId,
      remoteScenarioSetListFiltered,
      remoteDynamicModelListFiltered,
      remoteRecordListFiltered,
    } = store.studioConnector;

    const remoteResourceLength = remoteScenarioSetList.length
      + remoteDynamicModelList.length
      + remoteRecordList.length;

    const { tabs, currentKey } = this.state;

    return (
      <div className='data-profile'>
        <div className='data-profile-card'>
          <div className='card-header'>
            <span>Apollo Studio</span>
            <div className='data-profile-card_select_group'>
              {
                <div className='data-profile-card_select'>
                  <select
                    value={store.studioConnector.typeConditionValue}
                    onChange={this.onTypeSelectChange}
                  >
                    <option value='All'>All</option>
                    {remoteScenarioSetList.length > 0 && <option value='1'>Scenario profiles</option>}
                    {remoteDynamicModelList.length > 0 && <option value='2'>Dynamic model</option>}
                    {remoteRecordList.length > 0 && <option value='3'>Record profiles</option>}
                  </select>
                </div>
              }
              {remoteResourceLength > 0 && (<div className='data-profile-card_select'>
                <select
                  value={store.studioConnector.statusConditionValue}
                  onChange={this.onStatusSelectChange}
                >
                  <option value='All'>All</option>
                  <option value='notDownloaded'>Not downloaded</option>
                  <option value='downloaded'>Downloaded</option>
                  <option value='fail'>fail</option>
                  <option value='toBeUpdated'>To be updated</option>
                </select>
              </div>)}
            </div>
          </div>
          <div className='data-profile_scenario_set_column'>
            {/*no cerfiticate*/}
            {certificateStatus === 'notFound' && <ScenarioNoCertificate />}
            {/*scenario set list*/}
            <div className='scenario-set-list'>
              {certificateStatus === 'expired' && <ScenarioCertificateInvalid />}
              {/*场景集列表*/}
              {remoteScenarioSetListFiltered &&
                toJS(remoteScenarioSetListFiltered).map((item, index) => {
                  return (
                    <RemoteResourseItem
                      key={item.id}
                      item={item}
                    />
                  );
                })
              }
              {/*动力学模型列表*/}
              {remoteDynamicModelListFiltered &&
                toJS(remoteDynamicModelListFiltered).map((item, index) => {
                  return (
                    <RemoteResourseItem
                      key={item.id}
                      item={item}
                    />
                  );
                })
              }
              {/*数据包列表*/}
              {remoteRecordListFiltered &&
                toJS(remoteRecordListFiltered).map((item, index) => {
                  return (
                    <RemoteResourseItem
                      key={item.id}
                      item={item}
                    />
                  );
                })
              }
            </div>
          </div>
        </div>
        <div className='card'>
          <div className='card-header'>
            {
              tabs.map((tab, index) => {
                return (
                  <span
                    key={index}
                    className={currentKey === tab.key ? 'active' : ''}
                    onClick={() => {
                      this.setState({ currentKey: tab.key });
                      if (tab.key === 'recordProfiles') {
                        WS.checkWsConnection().loadLocalRecords();
                      }
                    }}
                  >{tab.title}</span>
                );
              })
            }
          </div>
          <div className='data-profile_profile_tabs_column'>
            {/*scenario set list*/}
            {currentKey === 'scenarioProfiles' && <div className='local-scenario-set-list'>
              <RadioGroup
                onChange={this.onRadioChange}
                value={currentScenarioSetId}
              >
                {toJS(scenarioSet).map((item, index) => {
                  return (
                    <LocalScenarioSetItem
                      key={item.scenarioSetId}
                      item={item}
                      currentScenarioSetId={currentScenarioSetId}
                    />
                  );
                })
                }
              </RadioGroup>
            </div>}

            {/*dynamic model*/}
            {currentKey === 'dynamicModel' && this.renderDynamicModelList()}
            {currentKey === 'recordProfiles' && this.renderRecordProfilesList()}
            {currentKey === 'vehicleProfiles' && this.renderVehicleProfilesList()}
            {currentKey === 'v2xProfiles' && this.renderV2xProfilesList()}
          </div>
        </div>
      </div>
    );
  }
}
;
