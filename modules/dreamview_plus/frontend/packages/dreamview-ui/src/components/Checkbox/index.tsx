import React from 'react';
import PropTypes from 'prop-types';
import './index.less';
import { CheckboxProps, Checkbox as InternalCheckBox } from 'antd';
import { CheckboxGroupProps } from 'antd/es/checkbox';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function CheckboxGroup(props: CheckboxGroupProps) {
    const { children, prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('check-box', customizePrefixCls);

    return <InternalCheckBox.Group prefixCls={prefixCls} {...rest} />;
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

CheckboxGroup.defaultProps = {};

CheckboxGroup.displayName = 'CheckboxGroup';
