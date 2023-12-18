import React from 'react';
import useComponentDisplay from '@dreamview/dreamview-core/src/hooks/useComponentDisplay';
import PlanningSpeed from './PlanningSpeed';
import PlanningAccelerationGraph from './PlanningAccelerationGraph';
import PlanningTheta from './PlanningTheta';
import PlanningKappa from './PlanningKappa';
import PlanningDKappa from './PlanningDKappa';
import PlanningReferenceTheta from './PlanningReferenceTheta';
import PlanningReferenceKappa from './PlanningReferenceKappa';
import PlanningReferenceDKappa from './PlanningReferenceDKappa';
import PlanningSTGraph from './PlanningSTGraph';
import PlanningSpeedHeuristicGraph from './PlanningSpeedHeuristicGraph';
import PlanningVTGraph from './PlanningVTGraph';
import ScenarioHistory from './ScenarioHistory';
import { IFilterList } from '../../useFilterList';
import CustomChart from './CustomChart';

const ComponentMap: Record<string, React.MemoExoticComponent<() => React.JSX.Element>> = {
    PlanningSpeed,
    PlanningAccelerationGraph,
    PlanningTheta,
    PlanningKappa,
    PlanningDKappa,
    PlanningReferenceTheta,
    PlanningReferenceKappa,
    PlanningReferenceDKappa,
    PlanningSTGraph,
    PlanningSpeedHeuristicGraph,
    PlanningVTGraph,
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

    const [displayStatus] = useComponentDisplay();
    return (
        <div>
            {displayStatus.isScenarioHistoryShow && (
                <>
                    <ScenarioHistory />
                    <Space />
                </>
            )}

            <CustomChart />

            {list.map(({ id: CompName }) => {
                const Comp: any = ComponentMap[CompName];
                if (!Comp) {
                    return null;
                }
                if (!checkStatus[CompName]) {
                    return null;
                }
                return (
                    <div key={CompName}>
                        <Comp key={CompName} />
                        <Space />
                    </div>
                );
            })}
        </div>
    );
}

export default React.memo(Chart);
