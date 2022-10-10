import React from 'react';
import { inject, observer } from 'mobx-react';
import { toJS } from 'mobx';

import { Radio } from 'antd';

import { ScenarioNoCertificate, ScenarioCertificateInvalid } from './ScenarioNoCertificate';
import ScenarioSetItem from './ScenarioSetItem';
import LocalScenarioSetItem from './LocalScenarioSetItem';
import LocalDynamicModelsItem from './LocalDynamicModelsItem';
import LocalRecordItem from './LocalRecordItem';
import WS, { PLUGIN_WS } from 'store/websocket';
import { throttle } from 'lodash';

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
      ],
    };
  }

  componentDidMount() {
    const { store } = this.props;
    setTimeout(() => {
      // 校验ws是否连接，确认链接后再校验证书
      PLUGIN_WS.checkWsConnection()
        .checkCertificate().downloadRecord();
      WS.checkWsConnection().loadLoocalScenarioSets();
      const {enableSimControl} = store.options;
      if (enableSimControl) {
        WS.getDymaticModelList();
      }
    }, 300);
  }

  onSelectChange = (e) => {
    const { store } = this.props;
    store.studioConnector.updateRemoteScenarioSelectConditionValue(e.target.value);
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
    const {enableSimControl} = store.options;
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
    const {enableSimControl} = store.options;
    if (enableSimControl) {
      return <div>Please close SimControl to switch the records player.</div>;
    }
    return (<div className='local-record-list'>
        {toJS(records).keys().map((item) => {
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

  render() {

    const { store } = this.props;

    const {
      certificateStatus,
      remoteScenarioSetList,
      scenarioSet,
      currentScenarioSetId,
      remoteScenarioSetListFiltered,
    } = store.studioConnector;

    const { tabs, currentKey } = this.state;

    return (
      <div className='data-profile'>
        <div className='data-profile-card'>
          <div className='card-header'>
            <span>Apollo Studio</span>
            {remoteScenarioSetList.length > 0 && (<div className='data-profile-card_select'>
              <select
                value={store.studioConnector.remoteScenarioSelectConditionValue}
                onChange={this.onSelectChange}
              >
                <option value='All'>All</option>
                <option value='notDownloaded'>Not downloaded</option>
                <option value='downloaded'>Downloaded</option>
                <option value='fail'>fail</option>
                <option value='toBeUpdated'>To be updated</option>
              </select>
            </div>)}
          </div>
          <div className='data-profile_scenario_set_column'>
            {/*no cerfiticate*/}
            { certificateStatus === 'notFound' && <ScenarioNoCertificate />}
            {/*scenario set list*/}
            <div className='scenario-set-list'>
              {certificateStatus === 'expired' && <ScenarioCertificateInvalid />}
              {toJS(remoteScenarioSetListFiltered).map((item, index) => {
                return (
                  <ScenarioSetItem
                    key={item.scenarioSetId}
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
          </div>
        </div>
      </div>
    );
  }
}
