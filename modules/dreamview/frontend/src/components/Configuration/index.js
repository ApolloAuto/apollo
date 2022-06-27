import React from 'react';
import { inject, observer } from 'mobx-react';
import { CONFIGURATION_WS } from 'store/websocket';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';
import _ from 'lodash';
import RegisterForm from './RegisterForm';
import VehicleProfile from './VehicleProfile';


@inject('store') @observer
export default class Configuration extends React.Component {

  constructor(props) {
    super(props);
    this.refreshConfigStatus = this.refreshConfigStatus.bind(this);
    this.onVehicleSnChange = this.onVehicleSnChange.bind(this);
  }

  updateVehicleSn(vehicleSn) {
    return CONFIGURATION_WS.updateVehicleSn(vehicleSn);
  }

  refreshConfigStatus() {
    return CONFIGURATION_WS.refreshConfigStatus();
  }

  onVehicleSnChange(evt) {
    this.updateVehicleSn(evt.value);
    this.refreshConfigStatus();
  }

  renderDefaultDkitProfiles() {
    const { configStatus, account } = this.props.store;
    // profiles now is hardcoded to 4 types of d-kit
    // TODO: use account's custom profiles and change default profiles as fallback
    const profiles = _.filter(configStatus.profiles, x => account.isVtypeBinded(x.type));
    return (
      <React.Fragment>
        {profiles.map(profile => (<VehicleConfigurtion
          key={profile.id}
          profileid={profile.id}
          name={profile.name}
          vtype={profile.type}
          status={profile.status}
        />))}
      </React.Fragment>
    );
  }

  renderVehicleBindGuid() {
    return (
      <div className="guide">
        <p>Your vehicle is not yet bound to your account,
          Please login and follow the instructions below to register in
          「
          <a className="apollo-studio" href="https://studio.apollo.auto/accounts" target="_blank">
            Apollo Studio
          </a>
          」
          manually.
        </p>
        <p>
          1. Click and open 「Apollo Studio」web application
        </p>
        <p>
          2. Visit 「My account」 page
        </p>
        <p>
          3. Register current vehicle in 「My vehicles」tab
        </p>
      </div>
    );
  }

  renderVehicleRegisterGuide() {
    // TODO: Show current vehicle info
    return (
      <div className="guide">
        <p>
          No binded vehicles, Please login and follow the instructions below to register in
          「
          <a className="apollo-studio" href="https://studio.apollo.auto/accounts" target="_blank">
            Apollo Studio
          </a>
          」
          manually.
        </p>
        <p>
          1. Click and open 「Apollo Studio」web application
        </p>
        <p>
          2. Visit 「My account」 page
        </p>
        <p>
          3. Register current vehicle in 「My vehicles」tab
        </p>
      </div>
    );
  }

  renderVehicleProfiles() {
    const { configStatus, account } = this.props.store;
    // profiles now is hardcoded to 4 types of d-kit
    // TODO: use account's custom profiles and change default profiles as fallback
    const profiles = configStatus.getCurrentVehicleProfiles();
    if (profiles.length === 0) {
      // no profile, create default virtual profile
      const vehicle = account.getVehicle(configStatus.vehicleSn);
      profiles.push({
        vehicleId: vehicle.id,
        vehicleType: vehicle.vtype,
        vehicleIn: vehicle.vin,
        status: 'UNKNOWN',
      });
    }
    // const profiles = _.filter(configStatus.profiles, x => account.isVtypeBinded(x.type));
    return (
      <React.Fragment>
        {profiles.map(profile => (<VehicleProfile
          key={profile.id}
          profileId={profile.id}
          name={profile.name}
          vehicleId={profile.vehicleId}
          vehicleType={profile.vehicleType}
          vehicleIn={profile.vehicleIn}
          status={profile.status}
        />))}
      </React.Fragment>
    );
  }

  render() {
    const { account, configStatus: { vehicleSn } } = this.props.store;
    return (
      <div className="card configuration">
        <Tabs>
          <TabList>
            <Tab
              onClick={this.refreshConfigStatus}
            >
              Vehicle Profiles
            </Tab>
          </TabList>
          <TabPanel>
            <RegisterForm vehicleSn={vehicleSn} onVehicleSnChange={this.onVehicleSnChange} />
            {!_.isNil(vehicleSn) && !_.isEmpty(vehicleSn)
              ? (account.isVehicleBinded(vehicleSn)
                ? this.renderVehicleProfiles()
                : this.renderVehicleBindGuid())
              : ''}
          </TabPanel>
        </Tabs>
      </div>
    );
  }
}
