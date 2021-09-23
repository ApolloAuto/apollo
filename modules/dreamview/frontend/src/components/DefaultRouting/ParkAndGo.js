import React from 'react';
import { observer } from 'mobx-react';

import RadioItem from 'components/common/RadioItem';

@observer
export default class ParkAndGo extends React.Component {
  render() {
    const { routeEditingManager, options } = this.props;
    const inRouteEditingMode = options.showRouteEditingBar;
    const entries = Object.keys(routeEditingManager.parkAndGoRoutings)
      .map((key, index) => (
        <RadioItem
          extraClasses={['default-routing-button']}
          key={`go_park_routing_${key}`}
          id="go_park_routing"
          title={key}
          onClick={() => {
            routeEditingManager.addDefaultRoutingPoint(key, false);
            if (!inRouteEditingMode) {
              routeEditingManager.currentDefaultRouting = key;
              options.showParkTimeInput = true;
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
