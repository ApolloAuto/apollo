import React from 'react';
import PropTypes from 'prop-types';
import { Radio as InternalRadio, RadioGroupProps } from 'antd';
// import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function RadioGroup(props: RadioGroupProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('radio-group', customizePrefixCls);
    return <InternalRadio.Group prefixCls={prefixCls} {...rest} />;
}

// Radio.propTypes = {
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

// Radio.defaultProps = {
//     backgroundColor: null,
//     primary: true,
//     size: 'medium',
//     onClick: undefined,
// };

RadioGroup.displayName = 'RadioGroup';
