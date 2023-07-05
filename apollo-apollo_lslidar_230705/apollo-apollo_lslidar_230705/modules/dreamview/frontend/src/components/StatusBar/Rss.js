import React from 'react';
import { inject, observer } from 'mobx-react';
import { MonitorItem } from 'components/Tasks/Console';
import { timestampMsToTimeString } from 'utils/misc';

@inject('store') @observer
export default class Rss extends React.Component {
  render() {
    const { monitor } = this.props;
    return (
            <div className="rss">
                <div className="rss-header"><div>RSS Info</div></div>
                <div className="rss-content-column">
                    <ul className="rss-console">
                        {monitor.rssInfo.map((item, index) => (
                            <MonitorItem
                                key={index}
                                text={item.msg}
                                level={item.logLevel}
                                time={timestampMsToTimeString(item.timestampMs)}
                            />
                        ))}
                    </ul>
                </div>
            </div>
    );
  }
}
