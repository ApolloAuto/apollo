import React, { useMemo, useState, useEffect, useRef, useCallback } from 'react';
import { Tabs, Checkbox, Popover, IconPark } from '@dreamview/dreamview-ui';
import CustomScroll from '@dreamview/dreamview-core/src/components/CustomScroll';
import { useTranslation } from 'react-i18next';
import { StreamDataNames } from '@dreamview/dreamview-core/src/services/api/types';
import { usePanelContext } from '@dreamview/dreamview-core/src/components/panels/base/store/PanelStore';
import Panel from '../base/Panel';
import { ENUM_PNCMONITOR_TAB } from './tab';
import useStyle from './useStyle';
import { useFilterList, IFilterList } from './useFilterList';
import useWebSocketServices from '../../../services/hooks/useWebSocketServices';
import Control from './Chart/Control';
import Planning from './Chart/Planning';
import Latency from './Chart/Latency';
import { PNCMonitorContext } from './PNCMonitorContext';
import usePlanningData, { initData as initPlanningData } from './usePlanningData';
import useControlData, { initData as initControlData } from './useControlData';
import useLatencyData, { initData as initLatenctData } from './useLatencyData';
import { useCreateViewBoxPosInfo, viewBoxPosInfoContext } from './Chart/ChartBase';

interface FilterProps {
    filterList: IFilterList;
    onToggle: (key: string) => void;
    checkStatus: Record<string, boolean>;
}
function Filter(props: FilterProps) {
    const { t } = useTranslation('pncMonitor');
    const { filterList, onToggle, checkStatus } = props;
    const { classes } = useStyle();

    const content = (
        <div className={classes['pnc-monitor-filter-content']}>
            {filterList.map((item) => (
                <div title={item.name} className={classes['pnc-monitor-filter-item']} key={item.name}>
                    <Checkbox title={item.name} onChange={() => onToggle(item.id)} checked={checkStatus[item.id]}>
                        {item.name}
                    </Checkbox>
                </div>
            ))}
        </div>
    );
    return (
        <Popover
            placement='bottomLeft'
            rootClassName={classes['pnc-monitor-filter-popover']}
            content={content}
            title={t('filtrate')}
        >
            <IconPark name='IcScreen' className={classes['pnc-monitor-filter']} />
        </Popover>
    );
}
const MemoFilter = React.memo(Filter);
function PncMonitorInner() {
    const { mainApi, isMainConnected } = useWebSocketServices();
    const consumerFuncs = useRef<any>([]);
    const { initSubscription, setKeyDownHandlers } = usePanelContext();

    const { classes } = useStyle();
    const [activeTab, setActiveTab] = useState<ENUM_PNCMONITOR_TAB>(ENUM_PNCMONITOR_TAB.PLANNING);
    const onTabChange = (key: string) => {
        setActiveTab(key as ENUM_PNCMONITOR_TAB);
    };

    const {
        current: { filterList, checkStatus, onToggle },
        planning,
        control,
    } = useFilterList(activeTab);

    const [{ planningData, controlData, latencyData }, setData] = useState<{
        planningData: any;
        controlData: any;
        latencyData: any;
    }>(() => ({
        planningData: initPlanningData(),
        controlData: initControlData(),
        latencyData: initLatenctData(),
    }));

    const { onSimData: onPlanningData } = usePlanningData();
    const { onSimData: onControlData, onRefresh: onClearControlView } = useControlData();
    const { onSimData: onLatencyData, onRefresh: onClearLatencyView } = useLatencyData();
    const { onRef, contextValue } = useCreateViewBoxPosInfo();

    useEffect(() => {
        initSubscription({
            [StreamDataNames.SIM_WORLD]: {
                consumer: (simData: any) => {
                    setData({
                        planningData: onPlanningData(simData),
                        latencyData: onLatencyData(simData),
                        controlData: onControlData(simData),
                    });
                },
            },
        });
    }, []);

    useEffect(() => {
        if (isMainConnected) {
            mainApi.triggerPncMonitor(true);
        }
    }, [mainApi]);

    const PNCMonitorContextValue = useMemo(
        () => ({
            onCosumerSimWrold(consumer: any) {
                consumerFuncs.current.push(consumer);
            },
            planningData,
            controlData,
            latencyData,
            onClearLatencyView,
            onClearControlView,
        }),
        [planningData, controlData, latencyData, onClearControlView, onClearLatencyView],
    );

    const items = [
        {
            key: ENUM_PNCMONITOR_TAB.PLANNING,
            label: 'Planning',
            forceRender: false,
            children: <Planning list={planning.filterList} checkStatus={planning.checkedStatus} />,
        },
        {
            key: ENUM_PNCMONITOR_TAB.CONTROL,
            forceRender: false,
            label: 'Control',
            children: <Control list={control.filterList} checkStatus={control.checkedStatus} />,
        },
        {
            key: ENUM_PNCMONITOR_TAB.LATENCY,
            forceRender: false,
            label: 'Latency',
            children: <Latency />,
        },
    ];

    return (
        <viewBoxPosInfoContext.Provider value={contextValue}>
            <CustomScroll ref={onRef} className={classes['pnc-container']}>
                <PNCMonitorContext.Provider value={PNCMonitorContextValue}>
                    <Tabs
                        destroyInactiveTabPane
                        onChange={onTabChange}
                        activeKey={activeTab}
                        rootClassName={classes.tabs}
                        inGray
                        items={items}
                    />
                    {activeTab !== ENUM_PNCMONITOR_TAB.LATENCY && (
                        <MemoFilter onToggle={onToggle} filterList={filterList} checkStatus={checkStatus} />
                    )}
                </PNCMonitorContext.Provider>
            </CustomScroll>
        </viewBoxPosInfoContext.Provider>
    );
}

PncMonitorInner.displayName = 'InternalPncMonitor';

function PncMonitor(props: any) {
    const Component = useMemo(
        () =>
            Panel({
                PanelComponent: PncMonitorInner,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.SIM_WORLD, needChannel: false }],
            }),
        [],
    );

    return <Component {...props} />;
}

export default React.memo(PncMonitor);
