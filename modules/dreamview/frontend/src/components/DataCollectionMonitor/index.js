import React from "react";
import { observer } from "mobx-react";
import classNames from "classnames";
@observer
export default class DataCollectionMonitor extends React.Component {
    render() {
        const { dataCollectionUpdateStatus, dataCollectionProgress } = this.props;

        if (!dataCollectionProgress || dataCollectionUpdateStatus.size === 0) {
            return <div className="no-data">No Data Found</div>;
        }

        let overallResult = null;
        if (dataCollectionProgress.has('Overall')) {
            overallResult = (
                <div key="Overall" className="category">
                    <div className="category-progress-background overall-progress">
                        <span className="category-progress"
                            style={{ width: dataCollectionProgress.get('Overall') + "%" }} />
                    </div>
                </div>
            );
        }

        const inProgressCategories = [];
        const completedCategories = [];
        dataCollectionUpdateStatus.forEach((isUpdated, description) => {
            if (description === 'Overall') {
                return;
            }

            const progressInPercentage = dataCollectionProgress.get(description);
            const element = (
                <div key={description} className="category">
                    <div className={classNames({
                        "category-description": true,
                        "category-updated": isUpdated,
                    })}>{`${description}:`}</div>
                    <div className="category-progress-background">
                        <span className="category-progress"
                            style={{ width: progressInPercentage + "%" }} />
                    </div>
                </div>
            );
            if (progressInPercentage >= 100.0) {
                completedCategories.push(element);
            } else {
                inProgressCategories.push(element);
            }
        });

        return (
            <div className="monitor data-collection-monitor">
                <hr className="section-divider" data-content="Overall Progress" />
                {overallResult}
                {inProgressCategories.length > 0 &&
                    <hr className="section-divider" data-content="Collecting Categories" />
                }
                {inProgressCategories}
                {completedCategories.length > 0 &&
                    <hr className="section-divider" data-content="Completed Categories" />
                }
                {completedCategories}
            </div>
        );
    }
}
