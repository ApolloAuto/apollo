import React from 'react';
import { observer } from 'mobx-react';
import classNames from 'classnames';

@observer
export default class ScenarioCollectionMonitor extends React.Component {
  renderCategoryProgress(description, progressInPercentage, isUpdated, isCompleted) {
    return (
            <div key={description} className="category">
                <div className={classNames({
                  'category-description': true,
                  'category-updated': isUpdated,
                })}
                >
                    {description}
                </div>
                <div className="category-progress-background">
                    <span
                        className={isCompleted
                          ? 'category-completed' : 'category-in-progress'}
                        style={{ width: `${progressInPercentage}%` }}
                    />
                </div>
            </div>
    );
  }

  render() {
    const { statusMap, progressMap, startUpdateProgress } = this.props;

    const inProgressCategories = [];
    const completedCategories = [];
    statusMap.forEach((isUpdated, description) => {
      const progressInPercentage = startUpdateProgress ? progressMap.get(description) : 0;
      const isCompleted = progressInPercentage >= 100.0;
      const element = this.renderCategoryProgress(
        description, progressInPercentage, isUpdated && startUpdateProgress, isCompleted);

      if (isCompleted) {
        completedCategories.push(element);
      } else {
        inProgressCategories.push(element);
      }
    });

    // To make sure the category in the last row has the width that
    // matches the rest, adding a dummy category that shows nothing
    // but helps flex-box adjusts the width correctly.
    if (inProgressCategories.length % 2 === 1) {
      inProgressCategories.push(<div className="dummy-category" key="in-progress-dummy" />);
    }
    if (completedCategories.length % 2 === 1) {
      completedCategories.push(<div className="dummy-category" key="completed-dummy" />);
    }

    return (
            <div className="scenario-container">
                <div className="category-container">
                    {inProgressCategories}
                </div>

                {completedCategories.length > 0
                    && (
                        <div className="category-container section-divider-on-top">
                            {completedCategories}
                        </div>
                    )}
            </div>
    );
  }
}
