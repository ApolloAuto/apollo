import React from 'react';
import { PopoverProps } from 'antd';
import { Popover } from '@dreamview/dreamview-ui';
import { makeStyles } from '@dreamview/dreamview-theme';

const useStyle = makeStyles((theme) => ({
    'custom-popover': {
        '&.dreamview-popover .dreamview-popover-inner': {
            padding: 0,
        },
        '& .dreamview-popover-inner,& .dreamview-popover-arrow::before, & .dreamview-popover-arrow::after': {
            color: '#A6B5CC',
            background: 'rgba(40, 43, 54)',
        },
        '& .dreamview-popover-arrow::before': {
            background: 'rgba(40, 43, 54)',
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(40, 43, 54)',
        },
        '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
            ...theme.tokens.typography.content,
            padding: '5px 10px',
        },
    },
}));

function CustomFuncPopover(props: PopoverProps) {
    const { classes, cx } = useStyle();
    return <Popover {...props} rootClassName={cx(classes['custom-popover'], props.rootClassName)} />;
}

export default React.memo(CustomFuncPopover);
