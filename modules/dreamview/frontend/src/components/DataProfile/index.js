import React from 'react';
import { inject, observer } from 'mobx-react';
import { toJS } from 'mobx';

import { Radio } from 'antd';

import { ScenarioNoCertificate, ScenarioCertificateInvalid } from './ScenarioNoCertificate';
import ScenarioSetItem from './ScenarioSetItem';
import LocalScenarioSetItem from './LocalScenarioSetItem';
import LocalDynamicModelsItem from './LocalDynamicModelsItem';
import WS, { PLUGIN_WS } from 'store/websocket';

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
                    onClick={() => this.setState({ currentKey: tab.key })}
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
          </div>
        </div>
      </div>
    );
  }
}
