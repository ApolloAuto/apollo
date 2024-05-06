import React, { useEffect } from 'react';
import { Button, Modal } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import { usePanelContext } from '../../../base/store/PanelStore';
import { useNavigate } from '../../../../../util/SimpleRouter';
import { useVizStore } from '../../VizStore';
import { useEventEmitter } from '../../../../../store/EventHandlersStore';
import { BusinessEventTypes } from '../../../../../store/EventHandlersStore/eventType';
import useWebSocketServices from '../../../../../services/hooks/useWebSocketServices';
import { DefaultPoint } from '../../../../../services/api/types';

export default function RoutingEditingOperationArea() {
    const EE = useEventEmitter();

    const navigate = useNavigate();

    const { classes, cx } = useStyle();

    const panelContext = usePanelContext();

    const { mainApi } = useWebSocketServices();

    const [{ routingEditor, routeManager }] = useVizStore();

    const { t } = useTranslation('routeEditing');

    const handleCancel = () => {
        Modal.confirm({
            content: t('cancelEditingRouting'),
            icon: <span />,
            onOk() {
                navigate('/');
                panelContext.exitFullScreen();
            },
            okText: t('modalConfirmYes'),
            cancelText: t('modalConfirmNo'),
        });
    };

    const handleSave = () => {
        const panelId = panelContext.panelId;
        const initialPoint: DefaultPoint = routingEditor.initiationMarker.initiationMarkerPosition;
        const wayPoint = routingEditor.pathwayMarker.pathWatMarkerPosition;
        const cycleNumber = routeManager.currentRouteMix.getCurrentRouteMix().currentRouteLoop?.currentRouteLoopTimes;
        const routeInfo = {
            initialPoint,
            wayPoint,
            cycleNumber,
        };
        if (!initialPoint) {
            mainApi.getStartPoint().then((res) => {
                routeInfo.initialPoint = res;
            });
        }
        const currentRouteMixValue = { currentRouteLoop: { currentRouteLoopState: false } };
        routeManager.currentRouteMix.setCurrentRouteMix(currentRouteMixValue);
        EE.emit(BusinessEventTypes.SimControlRoute, { panelId, routeInfo });
        navigate('/');
        panelContext.exitFullScreen();
    };

    return (
        <div className={classes['routing-editing-op-con']}>
            <Button ghost onClick={handleCancel}>{t('cancel')}</Button>
            <Button onClick={handleSave}>{t('saveEditing')}</Button>
        </div>
    );
}
