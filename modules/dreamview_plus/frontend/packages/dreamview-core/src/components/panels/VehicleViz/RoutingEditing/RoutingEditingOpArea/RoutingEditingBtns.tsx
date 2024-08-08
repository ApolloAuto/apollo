import React, { forwardRef } from 'react';
import { IconPark } from '@dreamview/dreamview-ui';
import { useTranslation } from 'react-i18next';
import useStyle from './useStyle';

interface RoutingEditingBtnProps {
    onClick?: () => void;
}

export const RoutingEditingBtn = forwardRef<HTMLDivElement, RoutingEditingBtnProps>(({ onClick }, ref) => {
    const { classes, cx } = useStyle();

    const { t } = useTranslation('routeEditing');
    return (
        <div
            id='routing-editing-btn'
            className={cx(classes['routing-editing-btn'], 'routing-editing-btn-disable')}
            onClick={onClick}
            ref={ref}
        >
            <i className={classes['routing-editing-btn__icon']}>
                <IconPark name='IcRoutingEditingEnter' />
            </i>
            <div className={classes['routing-editing-btn__text']}>{t('routingEditing')}</div>
        </div>
    );
});

export const CommonRoutingEditingBtn = forwardRef<HTMLDivElement, RoutingEditingBtnProps>(({ onClick }, ref) => {
    const { classes, cx } = useStyle();

    return (
        <div
            className={cx(classes['common-routing-editing-btn'], 'common-routing-editing-btn-disbale')}
            onClick={onClick}
            ref={ref}
        >
            <i className={classes['common-routing-editing-btn__icon']}>
                <IconPark name='IcCommonRoutin' />
            </i>
        </div>
    );
});

RoutingEditingBtn.displayName = 'RoutingEditingBtn';
CommonRoutingEditingBtn.displayName = 'CommonRoutingEditingBtn';
