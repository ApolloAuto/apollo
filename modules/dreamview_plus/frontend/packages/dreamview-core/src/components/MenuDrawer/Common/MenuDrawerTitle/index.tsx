import React from 'react';
import { useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';
import { CloseMenuAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { IconPark } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';

interface IMenuDrawerTitle {
    title: string | JSX.Element;
    border?: boolean;
    extra?: React.ReactNode;
}

function MenuDrawerTitle(props: IMenuDrawerTitle) {
    const { title, border = true, extra } = props;

    const { classes, cx } = useStyle();

    const [, dispatch] = useMenuStore();

    const onClose = () => {
        dispatch(CloseMenuAction());
    };

    return (
        <div className={cx(classes['menu-drawer-title'], { [classes['menu-drawer-title-border']]: border })}>
            <span>{title}</span>
            <div className={classes['menu-drawer-title-ic-container']}>
                <span className={classes['menu-drawer-title-ic']}>{extra}</span>
                &nbsp;&nbsp;&nbsp;
                <span className={classes['menu-drawer-title-ic']} onClick={onClose}>
                    <IconPark name='IcClose' />
                </span>
            </div>
        </div>
    );
}

export default React.memo(MenuDrawerTitle);
