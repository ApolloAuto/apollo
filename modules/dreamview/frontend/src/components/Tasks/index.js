import React from 'react';
import { inject, observer } from 'mobx-react';

import QuickStart from 'components/Tasks/QuickStart';
import Others from 'components/Tasks/Others';
import Delay from 'components/Tasks/Delay';
import Console from 'components/Tasks/Console';
import SensorCamera from 'components/Tasks/SensorCamera';

@inject('store') @observer
export default class Tasks extends React.Component {
  render() {
    const { options } = this.props;
    return (
            <div className="tasks">
                <QuickStart />
                <Others />
                <Delay />
                <Console />
                {(options.showVideo && !options.showPNCMonitor)
                    && <SensorCamera />}
            </div>
    );
  }
}
