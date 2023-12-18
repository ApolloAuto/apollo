import { useCallback, useMemo, useState } from 'react';
import { ENUM_PNCMONITOR_TAB } from './tab';

export type IFilterList = { name: string; id: string }[];

function useFilter(propFilterList: IFilterList) {
    const [filterList] = useState(propFilterList);
    const [checkedStatus, setChecked] = useState<Record<string, boolean>>(() =>
        propFilterList.reduce(
            (result, current) => ({
                ...result,
                [current.id]: true,
            }),
            {},
        ),
    );

    const checkList = useMemo(
        // promise the checkList order
        () => filterList.filter((item) => checkedStatus[item.id]).map((item) => item.id),
        [checkedStatus],
    );

    const onToggle = useCallback((key: string) => {
        setChecked((prev) => ({
            ...prev,
            [key]: !prev[key],
        }));
    }, []);

    return { filterList, checkedStatus, onToggle, checkList };
}

const planningFilter = [
    {
        id: 'PlanningSpeed',
        name: 'Planning Speed',
    },
    {
        id: 'PlanningAccelerationGraph',
        name: 'Planning Acceleration',
    },
    {
        id: 'PlanningTheta',
        name: 'Planning Theta',
    },
    {
        id: 'PlanningKappa',
        name: 'Planning Kappa',
    },
    {
        id: 'PlanningSpeedHeuristicGraph',
        name: 'Speed Heuristic',
    },
    {
        id: 'PlanningSTGraph',
        name: 'Planning S-T Graph',
    },
    {
        id: 'PlanningVTGraph',
        name: 'Planning V-T Graph',
    },
    {
        id: 'PlanningDKappa',
        name: 'Planning Kaappa Deravative',
    },
    {
        id: 'PlanningReferenceTheta',
        name: 'Reference Line Theta',
    },
    {
        id: 'PlanningReferenceKappa',
        name: 'Reference Line Kappa',
    },
    {
        id: 'PlanningReferenceDKappa',
        name: 'Reference Line Kapp Derivative',
    },
];

const controlFilter = [
    {
        id: 'TrajectoryGraph',
        name: 'Trajectory',
    },
    {
        id: 'SpeedGraph',
        name: 'Speed',
    },
    {
        id: 'AccelerationGraph',
        name: 'Acceleration',
    },
    {
        id: 'CurvatureGraph',
        name: 'Curvature',
    },
    {
        id: 'StationErrorGraph',
        name: 'Station error',
    },
    {
        id: 'LateralErrorGraph',
        name: 'Lateral error',
    },
    {
        id: 'HeadingErrorGraph',
        name: 'Heading error',
    },
];

export function useFilterList(key: ENUM_PNCMONITOR_TAB) {
    const planning = useFilter(planningFilter);
    const control = useFilter(controlFilter);

    const instance = {
        [ENUM_PNCMONITOR_TAB.PLANNING]: planning,
        [ENUM_PNCMONITOR_TAB.CONTROL]: control,
        [ENUM_PNCMONITOR_TAB.LATENCY]: {
            onToggle: () => false,
            filterList: [] as IFilterList,
            checkedStatus: {} as Record<string, boolean>,
        },
    }[key];

    const onToggle = useCallback(
        (id: string) => {
            instance.onToggle(id);
        },
        [key],
    );

    return {
        current: {
            filterList: instance.filterList,
            checkStatus: instance.checkedStatus,
            onToggle,
        },
        planning,
        control,
    };
}
