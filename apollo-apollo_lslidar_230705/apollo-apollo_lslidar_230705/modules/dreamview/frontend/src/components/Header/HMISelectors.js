import React from 'react';

import Selector from 'components/Header/Selector';
import ScenarioSetSelector from 'components/Header/ScenarioSetSelector';
import WS from 'store/websocket';
import { inject, observer } from 'mobx-react';
import { toJS } from 'mobx';

@inject('store') @observer
export default class HMISelectors extends React.Component {
  render() {

    const {
      dockerImage,
      modes, currentMode,
      maps, currentMap,
      vehicles, currentVehicle,
      isCoDriver,
      isMute,
    } = this.props.store.hmi;

    const enableSimControl = this.props.store.options.enableSimControl;

    const {
      scenarioSet,
      currentScenarioSetId,
      currentScenarioId
    } = this.props.store.studioConnector;

    const currentScenarioSet = scenarioSet.find(
      scenarioSetItem => scenarioSetItem.scenarioSetId === currentScenarioSetId
    ) || {};

    return (
            <React.Fragment>
                <Selector
                    name="setup mode"
                    options={modes}
                    currentOption={currentMode}
                    onChange={(event) => {
                      WS.changeSetupMode(event.target.value);
                    }}
                />
                <Selector
                    name="vehicle"
                    options={vehicles}
                    currentOption={currentVehicle}
                    onChange={(event) => {
                      WS.changeVehicle(event.target.value);
                    }}
                />
                <Selector
                    name="map"
                    options={maps}
                    currentOption={currentMap}
                    onChange={(event) => {
                      WS.changeMap(event.target.value);
                    }}
                />
              {
                (enableSimControl
                  && currentScenarioSet.scenarios
                  && currentScenarioSet.scenarios.length > 0)
                && (
                  <ScenarioSetSelector
                    name='scenarioSet'
                    options={
                      currentScenarioSet
                        .scenarios
                        .map(scenario => ({
                          value: scenario.scenarioId,
                          label: scenario.scenarioName,
                        }))
                    }
                    currentOption={currentScenarioId || ''}
                    onChange={(event) => {
                      WS.changeScenario(event.target.value);
                    }}
                  />)
              }
            </React.Fragment>
    );
  }
}
