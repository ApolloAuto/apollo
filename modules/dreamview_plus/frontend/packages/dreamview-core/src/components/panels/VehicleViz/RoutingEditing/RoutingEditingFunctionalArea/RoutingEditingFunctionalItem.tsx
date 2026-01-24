import React, { forwardRef, useMemo } from 'react';
import { IconPark } from '@dreamview/dreamview-ui';
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

    // @ts-ignore
    const { classes, cx } = useStyle({});

    const icon = useMemo(() => {
        switch (functionalName) {
            case FunctionalNameEnum.RELOCATE:
                return (
                    <i>
                        <IconPark name='IcLocation' />
                    </i>
                );

            case FunctionalNameEnum.WAYPOINT:
                return (
                    <i>
                        <IconPark name='IcWayPoint' />
                    </i>
                );

            case FunctionalNameEnum.LOOP:
                return (
                    <i>
                        <IconPark name='IcLoopRouting' />
                    </i>
                );

            case FunctionalNameEnum.FAVORITE:
                return (
                    <i>
                        <IconPark name='IcCommonRoutin' />
                    </i>
                );
            case FunctionalNameEnum.INDOOR_LOCALIZATION:
                return (
                    <i>
                        <IconPark name='IcIndoorIocation' />
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
            onClick={!disable ? onClick : () => null}
        >
            <div className={classes['routing-editing-functional__icon']}>{icon}</div>
        </div>
    );
}

export default forwardRef(RoutingEditingFunctionalItem);
