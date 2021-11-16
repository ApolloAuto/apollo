import React from 'react';
import { inject, observer } from 'mobx-react';

import CheckboxItem from 'components/common/CheckboxItem';
import StatusDisplay from 'components/ModuleController/StatusDisplay';

@inject('store') @observer
export default class ModuleController extends React.Component {
  render() {
    const {
      moduleStatus, componentStatus, allMonitoredComponentSuccess,
      isCalibrationMode, preConditionModule,
    } = this.props.store.hmi;

    const moduleEntries = Array.from(moduleStatus.keys()).sort().map((key) => (
            <CheckboxItem
                key={key}
                id={key}
                title={key}
                disabled={isCalibrationMode && (key === preConditionModule)
                  ? !allMonitoredComponentSuccess : false}
                isChecked={moduleStatus.get(key)}
                onClick={() => {
                  this.props.store.hmi.toggleModule(key);
                }}
                extraClasses="controller"
            />
    ));

    const componentEntries = Array.from(componentStatus.keys()).sort().map((key) => (
            <StatusDisplay
                key={key}
                title={key}
                status={componentStatus.get(key)}
            />
    ));

    return (
            <div className="module-controller">
                <div className="card">
                    <div className="card-header"><span>Components</span></div>
                    <div className="card-content-column">
                        {componentEntries}
                    </div>
                </div>
                <div className="card">
                    <div className="card-header"><span>Modules</span></div>
                    <div className="card-content-row">
                        {moduleEntries}
                    </div>
                </div>
            </div>
    );
  }
}
