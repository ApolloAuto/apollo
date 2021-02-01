import React from 'react';
import { observer } from 'mobx-react';

import RadioItem from 'components/common/RadioItem';

@observer
export default class DefaultRoutingPoint extends React.Component {
  render() {
    //Navigation mode not considered
    const { routeEditingManager, options } = this.props;
    const inRouteEditingMode = options.showRouteEditingBar;
    const entries = Object.keys(routeEditingManager.defaultRoutings)
      .map((key, index) => (
        <RadioItem
          extraClasses={['default-routing-button']}
          key={`default_routing_${key}`}
          id="default_routing"
          title={key}
          onClick={() => {
            routeEditingManager.addDefaultRoutingPoint(key);
            if (!inRouteEditingMode) {
              routeEditingManager.currentDefaultRouting = key;
              options.showCycleNumberInput = true;
            }
            options.showPOI = false;
          }}
          autoFocus={index === 0}
          checked={false}
        />
      ));

    return (
      <div className="card-content-row">{entries}</div>
    );
  }
}
