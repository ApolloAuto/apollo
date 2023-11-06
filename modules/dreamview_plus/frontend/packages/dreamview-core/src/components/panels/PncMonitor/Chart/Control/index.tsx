import React from 'react';
import HeadingErrorGraph from './HeadingErrorGraph';
import LateralErrorGraph from './LateralErrorGraph';
import StationErrorGraph from './StationErrorGraph';
import CurvatureGraph from './CurvatureGraph';
import AccelerationGraph from './AccelerationGraph';
import SpeedGraph from './SpeedGraph';
import TrajectoryGraph from './TrajectoryGraph';
import { IFilterList } from '../../useFilterList';

const ComponentMap: Record<string, React.MemoExoticComponent<() => React.JSX.Element>> = {
    HeadingErrorGraph,
    LateralErrorGraph,
    StationErrorGraph,
    CurvatureGraph,
    AccelerationGraph,
    SpeedGraph,
    TrajectoryGraph,
};

interface Chart {
    checkStatus: Record<string, boolean>;
    list: IFilterList;
}
function Space() {
    return <div style={{ height: '12px' }} />;
}
function Chart(props: Chart) {
    const { checkStatus, list } = props;
    return (
        <div>
            {list.map(({ id: CompName }) => {
                const Comp = ComponentMap[CompName];
                if (!Comp) {
                    return null;
                }
                return (
                    <div key={CompName} style={{ display: checkStatus[CompName] ? 'block' : 'none' }}>
                        <Comp key={CompName} />
                        <Space />
                    </div>
                );
            })}
        </div>
    );
}

export default React.memo(Chart);
