import React from 'react';
import PropTypes from 'prop-types';
import { Popover as InternalPopover, PopoverProps } from 'antd';
import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

const useStyle = makeStylesWithProps<{ classname: string }>()((theme, prop) => ({
    [prop.classname]: {
        '&.dreamview-popover .dreamview-popover-inner': {
            padding: '5px 10px',
            background: 'rgba(35,42,51, 0.8)',
        },
        '&.dreamview-popover .dreamview-popover-inner-content': {
            color: 'white',
            ...theme.tokens.typography.content,
        },
        '& .dreamview-popover-arrow::after': {
            background: 'rgba(35,42,51, 0.8)',
        },
        '& .dreamview-popover-arrow::before': {
            background: theme.tokens.colors.transparent,
        },
    },
}));


export function Popover(props: PopoverProps) {
    const { prefixCls: customizePrefixCls, rootClassName, ...rest } = props;
    const prefixCls = getPrefixCls('popover', customizePrefixCls);
    const { classes, cx } = useStyle({ classname: prefixCls });
    return <InternalPopover rootClassName={cx(classes[prefixCls], rootClassName)} prefixCls={prefixCls} {...rest} />;
}

Popover.propTypes = {
    // /**
    //  * Is this the principal call to action on the page?
    //  */
    // primary: PropTypes.bool,
    // /**
    //  * What background color to use
    //  */
    // backgroundColor: PropTypes.oneOf(['blue', 'red']),
    // /**
    //  * How large should the button be?
    //  */
    // size: PropTypes.oneOf(['small', 'medium', 'large']),
    // /**
    //  * Button contents
    //  */
    // label: PropTypes.string.isRequired,
    // /**
    //  * Optional click handler
    //  */
    // onClick: PropTypes.func,
};

Popover.defaultProps = {
    trigger: 'click',
};

Popover.displayName = 'Popover';
