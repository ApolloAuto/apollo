import React, { useEffect, useMemo, useState } from 'react';
import { apollo } from '@dreamview/dreamview';
import { useTranslation } from 'react-i18next';
import Panel from '../base/Panel';
import useStyle from './useStyle';
import { ComponentListItem, ComponentListProps } from './ComponentList';
import { StreamDataNames } from '../../../services/api/types';
import { usePanelContext } from '../base/store/PanelStore';
import EmptyPlaceHolder from '../base/EmptyPlaceHolder';
import CustomScroll from '../../CustomScroll';

type HMIStatus = apollo.dreamview.HMIStatus;

function InnerComponents() {
    const panelContext = usePanelContext();
    const { initSubscription, logger, setKeyDownHandlers } = panelContext;
    const { classes } = useStyle();
    const [hmi, setHmi] = useState<HMIStatus>();
    const { t: tPanels } = useTranslation('panels');

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

    return <CustomScroll className={classes['panel-components']}>{componentItems}</CustomScroll>;
}

InnerComponents.displayName = 'Components';

function Components(props: any) {
    const { t } = useTranslation('panels');

    const C = useMemo(
        () =>
            Panel({
                PanelComponent: InnerComponents,
                panelId: props.panelId,
                subscribeInfo: [{ name: StreamDataNames.HMI_STATUS, needChannel: false }],
                placeHolder: <EmptyPlaceHolder text={t('connectionError')} />,
            }),
        [],
    );

    return <C {...props} />;
}

export default React.memo(Components);
