import React from 'react';
import { useMenuStore } from '@dreamview/dreamview-core/src/store/MenuStore';
import { CloseMenuAction } from '@dreamview/dreamview-core/src/store/MenuStore/actions';
import { IconIcClose } from '@dreamview/dreamview-ui';
import useStyle from './useStyle';

interface IMenuDrawerTitle {
    title: string | JSX.Element;
    border?: boolean;
}

function MenuDrawerTitle(props: IMenuDrawerTitle) {
    const { title, border = true } = props;

    const { classes, cx } = useStyle();

    const [, dispatch] = useMenuStore();

    const onClose = () => {
        dispatch(CloseMenuAction());
    };

    return (
        <div className={cx(classes['menu-drawer-title'], { [classes['menu-drawer-title-border']]: border })}>
            <span>{title}</span>
            <span className={classes['menu-drawer-title-icclose']} onClick={onClose}>
                <IconIcClose />
            </span>
        </div>
    );
}

export default React.memo(MenuDrawerTitle);
