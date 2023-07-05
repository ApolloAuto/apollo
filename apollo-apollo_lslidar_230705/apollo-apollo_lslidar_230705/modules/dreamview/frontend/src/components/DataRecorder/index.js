import React from 'react';
import {
  Tab, Tabs, TabList, TabPanel,
} from 'react-tabs';

import DriveEventEditor from 'components/DataRecorder/DriveEventEditor';
import AudioEventEditor from 'components/DataRecorder/AudioEventEditor';

export default class DataRecorder extends React.Component {
  render() {
    return (
      <div className="card data-recorder">
        <Tabs>
            <TabList>
              <Tab>Add Drive Event</Tab>
              <Tab>Add Audio Event</Tab>
            </TabList>
            <TabPanel>
                <DriveEventEditor />
            </TabPanel>
            <TabPanel>
                <AudioEventEditor />
            </TabPanel>
        </Tabs>
      </div>
    );
  }
}
