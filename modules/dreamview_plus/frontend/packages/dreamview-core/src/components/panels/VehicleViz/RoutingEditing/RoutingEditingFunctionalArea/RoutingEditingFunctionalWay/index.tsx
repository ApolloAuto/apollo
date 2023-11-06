import React, { useEffect, useState } from 'react';
import { IconIcBackToAstPoint, IconIcRemoveAllPoints, Popover } from '@dreamview/dreamview-ui';
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
        <div className={cx(wayDisable && classes['functional-initial-disable'], classes['functional-initial-con'])}>
            <Popover
                content={t('removeLastPoint')}
                trigger='hover'
                rootClassName={classes['functional-initial-popover']}
            >
                <div className={wayDisable && cx(classes['functional-initial-every-icon-con'])}>
                    <div
                        className={cx(
                            'functional-initial-every-icon-disable',
                            classes['functional-initial-every-icon'],
                        )}
                        onClick={handleClickBack}
                    >
                        <IconIcBackToAstPoint />
                    </div>
                </div>
            </Popover>
            <Popover
                content={t('removeAllPoints')}
                trigger='hover'
                rootClassName={classes['functional-initial-popover']}
            >
                <div className={wayDisable && cx(classes['functional-initial-every-icon-con'])}>
                    <div
                        className={cx(
                            'functional-initial-every-icon-disable',
                            classes['functional-initial-every-icon'],
                        )}
                        onClick={handleClickRemove}
                    >
                        <IconIcRemoveAllPoints />
                    </div>
                </div>
            </Popover>
        </div>
    );
}
