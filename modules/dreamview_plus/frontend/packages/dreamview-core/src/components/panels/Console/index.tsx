import React, { useEffect, useMemo, useState } from 'react';
import type { apollo } from '@dreamview/dreamview';
import { IconPark } from '@dreamview/dreamview-ui';
import { timestampMsToTimeString } from '@dreamview/dreamview-core/src/util/misc';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import { usePanelContext } from '../base/store/PanelStore';
import { StreamDataNames } from '../../../services/api/types';
import Panel from '../base/Panel';
import CustomScroll from '../../CustomScroll';

type ISimulationWorld = apollo.dreamview.ISimulationWorld;

enum CONSOLE_LEVEL {
    ERROR = 'ERROR',
    FATAL = 'FATAL',
    FAIL = 'FAIL',
    SUCCESS = 'SUCCESS',
    UNKNOWN = 'UNKNOWN',
}

const LEVEL_CLASS = {
    [CONSOLE_LEVEL.ERROR]: 'error',
    [CONSOLE_LEVEL.FATAL]: 'error',
    [CONSOLE_LEVEL.FAIL]: 'error',
    [CONSOLE_LEVEL.SUCCESS]: 'info',
    [CONSOLE_LEVEL.UNKNOWN]: 'warn',
};

const LEVEL_ICON = {
    [CONSOLE_LEVEL.ERROR]: <IconPark name='IcErrorMessage' />,
    [CONSOLE_LEVEL.FATAL]: <IconPark name='IcErrorMessage' />,
    [CONSOLE_LEVEL.FAIL]: <IconPark name='IcErrorMessage' />,
    [CONSOLE_LEVEL.SUCCESS]: <IconPark name='IcRegularMessage' />,
    [CONSOLE_LEVEL.UNKNOWN]: <IconPark name='IcRegularMessage' />,
};

interface IMonitorItem {
    level: CONSOLE_LEVEL;
    text: string;
    time: string;
}
function MonitorItem(props: IMonitorItem) {
    const { classes, cx } = useStyle();
    const { level, text, time } = props;

    const icon = LEVEL_ICON[level] || <IconPark name='IcRegularMessage' />;
    const levelClass = LEVEL_CLASS[level] || 'warn';
    return (
        <li className={cx(classes['panel-console-monitor'], classes[levelClass])}>
            <span className={classes['panel-console-monitor-icon']}>{icon}</span>
            <span className={classes['panel-console-monitor-text']}>{text}</span>
            <span className={classes['panel-console-monitor-time']}>{time}</span>
        </li>
    );
}

const MonitorItemMemo = React.memo(MonitorItem);
function InternalConsole() {
    const panelContext = usePanelContext();
    const { data: subcribedData, initSubscription, setKeyDownHandlers } = panelContext;

    const [list, setList] = useState<IMonitorItem[]>([]);
    const [data, setData] = useState<ISimulationWorld>();
    const { t: tPanels } = useTranslation('panels');

    // 向list前端插入数据，最多保留29条
    const insertList = (item: IMonitorItem) => {
        setList((prevList) => {
            const newList = [item, ...prevList];
            if (newList.length > 29) {
                newList.pop();
            }
            return newList;
        });
    };

    useEffect(() => {
        initSubscription({
            [StreamDataNames.SIM_WORLD]: {
                consumer: (simData) => {
                    setData(simData);
                },
            },
        });
    }, []);

    useEffect(() => {
        data?.notification?.forEach((item) => {
            insertList({
                level: item.item.logLevel as any,
                text: item.item.msg,
                time: timestampMsToTimeString(item.timestampSec * 1000),
            });
        });
    }, [data]);

    const { classes } = useStyle();
    return (
        <CustomScroll className={classes['panel-console-root']}>
            <div className={classes['panel-console-inner']}>
                {list.map((item, index) => (
                    <MonitorItemMemo key={index + 1} text={item.text} level={item.level} time={item.time as any} />
                ))}
            </div>
        </CustomScroll>
    );
}

function Console(props: any) {
    const C = useMemo(
        () =>
            Panel({
                PanelComponent: InternalConsole,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.SIM_WORLD, needChannel: false }],
            }),
        [],
    );

    return <C {...props} />;
}

InternalConsole.displayName = 'InternalConsole';

export default React.memo(Console);
