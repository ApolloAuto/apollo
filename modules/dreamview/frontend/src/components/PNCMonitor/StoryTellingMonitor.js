
import React from "react";
import { inject, observer } from "mobx-react";
import classNames from "classnames";
import { Tabs, TabList, Tab, TabPanel } from "react-tabs";

class StoryItem extends React.PureComponent {
    render() {
        const { name, value } = this.props;

        const textClassNames = classNames({
            "text": true,
            "active": value,
        });

        return (
            <tr className="monitor-table-item">
                <td className={textClassNames}>{name.toUpperCase()}</td>
                <td className={textClassNames}>{value ? 'YES' : 'No'}</td>
            </tr>
        );
    }
}

@inject("store") @observer
export default class StoryTellingMonitor extends React.Component {
    render() {
        const { stories } = this.props.store.storyTellers;

        let storyTable = null;
        if (stories.size > 0) {
            storyTable = stories.entries().map(([story, isOn]) =>
                <StoryItem key={`story_${story}`} name={story} value={isOn} />
            );
        } else {
            storyTable = (
                <tr className="monitor-table-item">
                    <td className="text">No Data</td>
                </tr>
            );
        }

        return (
            <Tabs>
                <TabList>
                    <Tab>Story Tellers</Tab>
                </TabList>
                <TabPanel className="monitor-table-container">
                    <table className="monitor-table">
                        <tbody>{storyTable}</tbody>
                    </table>
                </TabPanel>
            </Tabs>
        );
    }
}
