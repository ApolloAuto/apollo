import React from "react";
import { observer } from "mobx-react";

@observer
export default class DataCollectionMonitor extends React.Component {
    render() {
        const { dataCollectionProgress } = this.props;

        const categoryElements = [];
        dataCollectionProgress.forEach((progressInPercentage, description) => {
            categoryElements.push(
                <div key={description} className="category">
                    <span className="category-progress"
                          style={{ width: progressInPercentage + "%" }} />
                    <span className="category-description">{description}</span>
                    <span className="category-percentage">
                        {`${progressInPercentage.toFixed(2)} %`}
                    </span>
                </div>
            );
        });

        return (
            <div className="monitor data-collection-monitor">
                {categoryElements.length !== 0
                    ? categoryElements
                    : <div className="no-data">No Data Found</div>
                }
            </div>
        );
    }
}
