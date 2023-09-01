import { getPrefixCls } from '@dreamview/dreamview-ui/src/tools/prefixCls/prefixCls';
import React, { HTMLAttributes, PropsWithChildren, forwardRef } from 'react';
import classNames from 'classnames';
import './index.less';

type PanelRoot = HTMLAttributes<HTMLDivElement>;

export const PanelRoot = forwardRef<HTMLDivElement, PropsWithChildren<PanelRoot>>((props, ref): JSX.Element => {
    const { className, ...rest } = props;
    const prefixCls = getPrefixCls('panel-root');
    const classes = classNames(prefixCls, className);

    return (
        <div ref={ref} className={classes} {...rest}>
            {props.children}
        </div>
    );
});

PanelRoot.displayName = 'PanelRoot';
