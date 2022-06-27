import { observable, action, computed } from 'mobx';
import WS from 'store/websocket';
import { ACCOUNT_WS } from 'store/websocket';
import _ from 'lodash';

export default class ConfigProfileStatus {

  @observable vehicleSn = '';

  @observable profiles = [];

  @action update(status) {
    let configChanged = false;
    const cIDList = _.map(this.profiles, x => x.id);
    const nIDList = _.map(status.profiles, x => x.id);
    const idList = _.uniq([...cIDList, ...nIDList]);
    for (let i = 0; i < idList.length; ++i) {
      const id = idList[i];
      const curProfile = _.find(this.profiles, x => x.id === id);
      const newProfile = _.find(status.profiles, x => x.id === id) ;
      if (_.isNil(curProfile) || _.isNil(newProfile)) {
        configChanged = true;
        break;
      }
      const isFetching = _.includes(
        ['SYNCING', 'DOWNLOADING', 'UPLOADING', 'RESETING'],
        newProfile.status,
      );
      if (!isFetching && curProfile.status !== newProfile.status) {
        configChanged = true;
        break;
      }
    }
    if (this.vehicleSn !== status.vehicleIn) {
      configChanged = true;
      this.vehicleSn = status.vehicleIn;
    }
    if (!OFFLINE_PLAYBACK && configChanged) {
      WS.reloadVehicles();
    }
    this.profiles = status.profiles;
  }

  @action forceAccountInfoReload() {
    ACCOUNT_WS.refreshAccountInfo();
  }

  getCurrentVehicleProfiles() {
    return _.filter(this.profiles, (item) => {
      return item.vehicleIn === this.vehicleSn;
    });
  }
}
