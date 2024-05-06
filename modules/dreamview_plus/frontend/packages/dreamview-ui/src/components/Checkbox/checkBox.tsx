import React from 'react';
import PropTypes from 'prop-types';
import './index.less';
import { CheckboxProps, Checkbox as InternalCheckBox } from 'antd';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Checkbox(props: CheckboxProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('check-box', customizePrefixCls);

    return <InternalCheckBox prefixCls={prefixCls} {...rest} />;
}

// Checkbox.propTypes = {
//     /**
//      * Is this the principal call to action on the page?
//      */
//     primary: PropTypes.bool,
//     /**
//      * What background color to use
//      */
//     backgroundColor: PropTypes.oneOf(['blue', 'red']),
//     /**
//      * How large should the button be?
//      */
//     size: PropTypes.oneOf(['small', 'medium', 'large']),
//     /**
//      * Button contents
//      */
//     label: PropTypes.string.isRequired,
//     /**
//      * Optional click handler
//      */
//     onClick: PropTypes.func,
// };

Checkbox.defaultProps = {};

Checkbox.displayName = 'Checkbox';
