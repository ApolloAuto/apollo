import { getPrefixCls } from '@dreamview/dreamview-ui/src/tools/prefixCls/prefixCls';
import React, { forwardRef, HTMLAttributes, PropsWithChildren, useMemo } from 'react';
import classNames from 'classnames';
import './index.less';

type FullScreenProps = {
    enabled: boolean;
} & HTMLAttributes<HTMLDivElement>;

export const FullScreen = forwardRef<HTMLDivElement, PropsWithChildren<FullScreenProps>>((props, ref): JSX.Element => {
    const { enabled, className, ...rest } = props;
    const prefixCls = getPrefixCls('full-screen');
    const classes = classNames(`${prefixCls}-container`, className);

    const style: React.CSSProperties = useMemo(() => {
        if (enabled) {
            return {
                position: 'fixed',
                top: 0,
                bottom: 0,
                left: 0,
                right: 0,
                zIndex: 999999999999,
                backgroundColor: 'rgba(0, 0, 0, 1)',
            };
        }

        return undefined;
    }, [enabled]);

    return (
        <div ref={ref} className={classes} style={style} {...rest}>
            {props.children}
        </div>
    );
});

FullScreen.displayName = 'FullScreen';
