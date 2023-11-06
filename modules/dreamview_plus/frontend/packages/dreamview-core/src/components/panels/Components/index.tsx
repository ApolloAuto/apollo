import React, { useEffect, useMemo, useState } from 'react';
import { apollo } from '@dreamview/dreamview';
import Panel from '../base/Panel';
import useStyle from './useStyle';
import { ComponentListItem, ComponentListProps } from './ComponentList';
import { StreamDataNames } from '../../../services/api/types';
import { usePanelContext } from '../base/store/PanelStore';

type HMIStatus = apollo.dreamview.HMIStatus;

function InnerComponents() {
    const panelContext = usePanelContext();
    const { initSubscription, logger } = panelContext;
    const { classes } = useStyle();
    const [hmi, setHmi] = useState<HMIStatus>();

    useEffect(() => {
        initSubscription({
            [StreamDataNames.HMI_STATUS]: {
                consumer: (data) => {
                    setHmi(data as HMIStatus);
                },
            },
        });
    }, []);

    const componentItems = useMemo(() => {
        if (!hmi?.monitoredComponents) {
            return null;
        }

        return Array.from(Object.keys(hmi?.monitoredComponents))
            ?.sort()
            ?.map((key) => (
                <ComponentListItem
                    key={key}
                    name={key}
                    status={hmi?.monitoredComponents?.[key]?.status as unknown as string}
                />
            ));
    }, [hmi]);

    return <div className={classes['panel-components']}>{componentItems}</div>;
}

InnerComponents.displayName = 'Components';

function Components(props: any) {
    const C = useMemo(
        () =>
            Panel({
                PanelComponent: InnerComponents,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.HMI_STATUS, needChannel: false }],
            }),
        [],
    );

    return <C {...props} />;
}

export default React.memo(Components);
