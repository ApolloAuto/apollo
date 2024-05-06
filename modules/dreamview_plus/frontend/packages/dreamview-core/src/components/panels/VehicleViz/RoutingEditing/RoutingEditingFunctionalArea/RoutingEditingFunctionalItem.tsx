import React, { forwardRef, useMemo } from 'react';
import { IconIcCommonRoutin, IconIcLocation, IconIcLoopRouting, IconIcWayPoint } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';
import { FunctionalNameEnum } from './types';

interface IRoutingEditingFunctionalItemProps {
    functionalName: FunctionalNameEnum;
    checkedItem: FunctionalNameEnum;
    onClick?: () => void;
    children?: React.ReactNode;
    disable?: boolean;
}
function RoutingEditingFunctionalItem(
    { functionalName, checkedItem, onClick, disable }: IRoutingEditingFunctionalItemProps,
    ref: React.Ref<HTMLDivElement>,
) {
    const checked = useMemo(() => checkedItem === functionalName, [functionalName, checkedItem]);

    const { classes, cx } = useStyle({});

    const icon = useMemo(() => {
        switch (functionalName) {
            case FunctionalNameEnum.RELOCATE:
                return (
                    <i>
                        <IconIcLocation />
                    </i>
                );

            case FunctionalNameEnum.WAYPOINT:
                return (
                    <i>
                        <IconIcWayPoint />
                    </i>
                );

            case FunctionalNameEnum.LOOP:
                return (
                    <i>
                        <IconIcLoopRouting />
                    </i>
                );

            case FunctionalNameEnum.FAVORITE:
                return (
                    <i>
                        <IconIcCommonRoutin />
                    </i>
                );
            default:
                return null;
        }
    }, [functionalName]);

    return (
        <div
            ref={ref}
            className={cx(classes['routing-editing-functional__item'], {
                [classes['routing-editing-functional__item--active']]: checked,
                [classes['hover-color-change']]: !checked,
                [classes['routing-editing-functional__item--disable']]: disable,
            })}
            onClick={onClick}
        >
            <div className={classes['routing-editing-functional__icon']}>{icon}</div>
        </div>
    );
}

export default forwardRef(RoutingEditingFunctionalItem);
