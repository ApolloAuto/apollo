import React, { useEffect, useState } from 'react';
import { IconPark, Popover } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';
import { useVizStore } from '../../../VizStore';

interface RoutingEditingFunctionalWayProps {}

export default function RoutingEditingFunctionalWay(props: RoutingEditingFunctionalWayProps) {
    const { classes, cx } = useStyle();

    const [{ routingEditor }] = useVizStore();

    const { t } = useTranslation('routeEditing');

    const [wayPointCount, setWayPointCount] = useState(routingEditor.pathwayMarker.positionsCount);

    const [wayDisable, setWayDisable] = useState(!(wayPointCount > 0));

    const handleClickBack = () => {
        const backToAstPoint = routingEditor.pathwayMarker.undo();
        setWayPointCount(routingEditor.pathwayMarker.positionsCount);
    };

    const handleClickRemove = () => {
        routingEditor.pathwayMarker.reset();
        setWayPointCount(routingEditor.pathwayMarker.positionsCount);
    };

    useEffect(() => {
        setWayDisable(!wayPointCount);
    }, [wayPointCount]);

    return (
        <div className={cx({ [classes['functional-initial-disable']]: wayDisable }, classes['functional-initial-con'])}>
            <Popover
                content={t('removeLastPoint')}
                trigger='hover'
                rootClassName={classes['functional-initial-popover']}
            >
                <div className={cx({ [classes['functional-initial-every-icon-con']]: wayDisable })}>
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
                content={t('removeAllPoints')}
                trigger='hover'
                rootClassName={classes['functional-initial-popover']}
            >
                <div className={cx({ [classes['functional-initial-every-icon-con']]: wayDisable })}>
                    <div
                        className={cx(
                            'functional-initial-every-icon-disable',
                            classes['functional-initial-every-icon'],
                        )}
                        onClick={handleClickRemove}
                    >
                        <IconPark name='IcRemoveAllPoints' />
                    </div>
                </div>
            </Popover>
        </div>
    );
}
