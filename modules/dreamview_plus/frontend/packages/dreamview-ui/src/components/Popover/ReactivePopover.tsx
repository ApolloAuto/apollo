import React from 'react';
import PropTypes from 'prop-types';
import { Popover as InternalPopover, PopoverProps } from 'antd';
import { useMakeStyle } from '@dreamview/dreamview-theme';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

function useStyle(classname: string) {
    const hoc = useMakeStyle((theme) => ({
        [classname]: {
            boxShadow: theme.components.reactivePopover.boxShadow,
            '& .dreamview-operate-popover-content .dreamview-operate-popover-inner': {
                padding: '12px 0',
                background: `${theme.components.reactivePopover.bgColor}`,
            },
            '& .dreamview-operate-popover-content .dreamview-operate-popover-inner-content': {
                ...theme.tokens.typography.content,
                color: theme.components.reactivePopover.color,
            },
            '& .dreamview-operate-popover-arrow::after': {
                background: theme.components.reactivePopover.bgColor,
            },
            '& .dreamview-operate-popover-arrow::before': {
                background: theme.tokens.colors.transparent,
            },
        },
    }));

    return hoc();
}

export function ReactivePopover(props: PopoverProps) {
    const { prefixCls: customizePrefixCls, rootClassName, ...rest } = props;
    const prefixCls = getPrefixCls('operate-popover', customizePrefixCls);
    const { classes, cx } = useStyle(prefixCls);
    return <InternalPopover rootClassName={cx(classes[prefixCls], rootClassName)} prefixCls={prefixCls} {...rest} />;
}

ReactivePopover.displayName = 'ReactivePopover';
