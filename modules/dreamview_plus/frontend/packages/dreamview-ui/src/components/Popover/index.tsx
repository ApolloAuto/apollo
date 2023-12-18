import React from 'react';
import PropTypes from 'prop-types';
import { Popover as InternalPopover, PopoverProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Popover(props: PopoverProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('popover', customizePrefixCls);

    return <InternalPopover prefixCls={prefixCls} {...rest} />;
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
