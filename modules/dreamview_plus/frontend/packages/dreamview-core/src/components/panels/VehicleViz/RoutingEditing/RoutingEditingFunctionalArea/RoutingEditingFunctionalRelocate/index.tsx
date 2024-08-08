import React, { useEffect, useState } from 'react';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import { useVizStore } from '../../../VizStore';
import useWebSocketServices from '../../../../../../services/hooks/useWebSocketServices';

interface RoutingEditingFunctionalRelocateProps {}

export default function RoutingEditingFunctionalRelocate(props: RoutingEditingFunctionalRelocateProps) {
    const { classes, cx } = useStyle();

    const [{ routingEditor }] = useVizStore();

    const { t } = useTranslation('routeEditing');

    const { isMainConnected, mainApi } = useWebSocketServices();

    const [initPointCount, setInitPointCount] = useState(routingEditor.initiationMarker.positionsCount);

    const [relocateDisable, setRelocateDisable] = useState(!(initPointCount > 0));

    const handleClickBack = () => {
        const backToAstPoint = routingEditor.initiationMarker.undo();
        if (isMainConnected) {
            if (!backToAstPoint) {
                mainApi.setResetPoint().then(() => {
                    setInitPointCount(routingEditor.initiationMarker.positionsCount);
                });
            } else {
                mainApi.setStartPoint({ point: backToAstPoint }).then(() => {
                    setInitPointCount(routingEditor.initiationMarker.positionsCount);
                });
            }
        }
    };

    const handleClickReset = () => {
        if (isMainConnected) {
            mainApi.setResetPoint().then(() => {
                routingEditor.initiationMarker.reset();
                setInitPointCount(routingEditor.initiationMarker.positionsCount);
            });
        }
    };

    useEffect(() => {
        setRelocateDisable(!initPointCount);
    }, [initPointCount]);

    return (
        <div
            className={cx(relocateDisable && classes['functional-initial-disable'], classes['functional-initial-con'])}
        >
            <Popover
                content={t('backToLastPoint')}
                trigger='hover'
                rootClassName={classes['functional-initial-popover']}
            >
                <div className={relocateDisable && cx(classes['functional-initial-every-icon-con'])}>
                    <div
                        className={cx(
                            'functional-initial-every-icon-disable',
                            classes['functional-initial-every-icon'],
                        )}
                        onClick={handleClickBack}
                    >
                        <IconPark name='IcBackToAstPoint' />
                    </div>
                </div>
            </Popover>
            <Popover
                content={t('backToStartPoint')}
                trigger='hover'
                rootClassName={classes['functional-initial-popover']}
            >
                <div className={relocateDisable && cx(classes['functional-initial-every-icon-con'])}>
                    <div
                        className={cx(
                            'functional-initial-every-icon-disable',
                            classes['functional-initial-every-icon'],
                        )}
                        onClick={handleClickReset}
                    >
                        <IconPark name='IcBackTheStartingPoint' />
                    </div>
                </div>
            </Popover>
        </div>
    );
}
