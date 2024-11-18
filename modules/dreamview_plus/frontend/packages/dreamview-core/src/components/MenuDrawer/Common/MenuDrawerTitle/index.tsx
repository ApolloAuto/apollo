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

    const isArr = Array.isArray(extra);

    const { classes, cx } = useStyle();

    const [, dispatch] = useMenuStore();

    const onClose = () => {
        dispatch(CloseMenuAction());
    };

    return (
        <div className={cx(classes['menu-drawer-title'], { [classes['menu-drawer-title-border']]: border })}>
            <span>{title}</span>
            <div className={classes['menu-drawer-title-ic-container']}>
                {isArr ? (
                    <>
                        {extra.map((item, index) => (
                            <>
                                <span key={`${index + 1}`} className={classes['menu-drawer-title-ic']}>
                                    {item}
                                </span>
                                &nbsp;&nbsp;&nbsp;
                            </>
                        ))}
                    </>
                ) : (
                    <>
                        <span className={classes['menu-drawer-title-ic']}>{extra}</span>
                        &nbsp;&nbsp;&nbsp;
                    </>
                )}
                <span className={classes['menu-drawer-title-ic']} onClick={onClose}>
                    <IconPark name='IcClose' />
                </span>
            </div>
        </div>
    );
}

export default React.memo(MenuDrawerTitle);
