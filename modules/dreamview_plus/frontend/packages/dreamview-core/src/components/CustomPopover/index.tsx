import React from 'react';
import { PopoverProps } from 'antd';
import { Popover } from '@dreamview/dreamview-ui';
import { useMakeStyle } from '@dreamview/dreamview-theme';

function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'custom-popover': {
            '& .dreamview-popover-inner,& .dreamview-popover-arrow::before, & .dreamview-popover-arrow::after': {
                background: 'rgba(61, 67, 78, 0.8)',
            },
            '& .dreamview-popover-arrow::before': {
                background: 'rgba(61, 67, 78, 0.8)',
            },
            '& .dreamview-popover-arrow::after': {
                background: 'rgba(61, 67, 78, 0.8)',
            },
            '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
                ...theme.tokens.typography.content,
                padding: '5px 10px',
            },
        },
    }));
    return hoc();
}

export default function CustomPopover(props: PopoverProps) {
    const { classes, cx } = useStyle();
    return <Popover {...props} rootClassName={cx(classes['custom-popover'], props.rootClassName)} />;
}
