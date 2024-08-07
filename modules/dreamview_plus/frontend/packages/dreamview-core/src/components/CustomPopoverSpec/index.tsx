import React from 'react';
import { PopoverProps } from 'antd';
import { Popover } from '@dreamview/dreamview-ui';
import { makeStyles } from '@dreamview/dreamview-theme';

const useStyle = makeStyles((theme) => ({
    'custom-popover': {
        '&.dreamview-popover .dreamview-popover-inner': {
            padding: 0,
        },
        '& .dreamview-popover-inner, & .dreamview-popover-arrow::after': {
            background: 'rgba(80, 88, 102, 0.8)',
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(80, 88, 102, 0.8)',
        },
        '& .dreamview-popover-content .dreamview-popover-inner .dreamview-popover-inner-content': {
            ...theme.tokens.typography.content,
        },
    },
}));
function CustomPopover(props: PopoverProps) {
    const { classes, cx } = useStyle();
    return <Popover {...props} rootClassName={`${classes['custom-popover']} ${props.rootClassName || ''}`} />;
}

export default React.memo(CustomPopover);
