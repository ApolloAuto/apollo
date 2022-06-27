import { observable, action, computed } from 'mobx';
import _ from 'lodash';

export default class AccountInfo {
  @observable id = '';

  @observable username = '';

  @observable vehicles = [];

  @observable profiles = [];

  @computed get isLogged() {
    return this.username !== '' && !_.isNil(this.username);
  }

  @computed get hasBindedVehicle() {
    return !_.isNil(this.vehicles) && this.vehicles.length > 0;
  }

  getVehicle(vehicleSn) {
    const vehicle = _.find(this.vehicles, (item) => {
      return item.vin === vehicleSn;
    });
    return vehicle;
  }

  getVehicleProfiles(vehicleSn) {
    const vehicle = this.getVehicle(vehicleSn);
    if (!vehicle) {
      return [];
    }
    const profiles = _.filter(this.profiles, (profile) => {
      return profile.vehicleId === vehicle.id;
    });
    return profiles;
  }

  isVehicleBinded(vin) {
    return _.some(this.vehicles, x => x.vin === vin);
  }

  isVtypeBinded(vtype) {
    return _.some(this.vehicles, x => x.vtype === vtype);
  }

  @action update(info) {
    this.username = info.username;
    this.vehicles = info.vehicles;
    this.profiles = info.profiles;
  }
}
