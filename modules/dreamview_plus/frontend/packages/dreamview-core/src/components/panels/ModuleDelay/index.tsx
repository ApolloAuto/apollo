import React, { useEffect, useMemo, useState } from 'react';
import type { apollo } from '@dreamview/dreamview';
import { useTranslation } from 'react-i18next';
import { millisecondsToTime } from '../../../util/misc';
import useStyle from './useStyle';
import { StreamDataNames } from '../../../services/api/types';
import { usePanelContext } from '../base/store/PanelStore';
import Panel from '../base/Panel';
import CustomScroll from '../../CustomScroll';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;

interface IModuleDelayMsg {
    name: string;
    delay: number | string;
}

function InternalModuleDelay() {
    const panelContext = usePanelContext();
    const { initSubscription, logger, data: subcribedData, setKeyDownHandlers } = panelContext;
    const { classes, cx } = useStyle();

    const [delay, setDelay] = useState<Record<string, number>>({});
    const [data, setData] = useState<ISimulationWorld>();

    useEffect(() => {
        initSubscription({
            [StreamDataNames.SIM_WORLD]: {
                consumer: (simData) => {
                    setData(simData);
                },
            },
        });
    }, []);

    const delayList = useMemo(() => {
        if (delay) {
            return Object.keys(delay)
                .sort()
                .map((key) => {
                    const warning = Number(delay[key]) > 2000 && key !== 'TrafficLight';
                    return {
                        name: key,
                        delay: delay[key],
                        fronDelay: (delay[key] as any) < 0 ? '-' : millisecondsToTime(delay[key] as number),
                        frotWarning: warning,
                    };
                });
        }

        return [];
    }, [delay]);

    useEffect(() => {
        setDelay(data?.delay);
    }, [data]);

    return (
        <CustomScroll className={classes['panel-module-delay-root']}>
            <ul className={classes['panel-module-delay-scroll']}>
                {delayList.map((item, index) => (
                    <li className={classes['panel-module-delay-item']} key={index + 1}>
                        <span className={classes.name}>{item.name}</span>
                        <span className={cx(classes.time, { [classes.error]: item.frotWarning })}>
                            {item.fronDelay}
                        </span>
                    </li>
                ))}
            </ul>
        </CustomScroll>
    );
}

function ModuleDelay(props: any) {
    const Component = useMemo(
        () =>
            Panel({
                PanelComponent: InternalModuleDelay,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.SIM_WORLD, needChannel: false }],
            }),
        [],
    );

    return <Component {...props} />;
}

InternalModuleDelay.displayName = 'InternalModuleDelay';

export default React.memo(ModuleDelay);
