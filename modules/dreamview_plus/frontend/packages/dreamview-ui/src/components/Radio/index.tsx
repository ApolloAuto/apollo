import React from 'react';
import PropTypes from 'prop-types';
import { Radio as InternalRadio, RadioProps } from 'antd';
// import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Radio(props: RadioProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('radio', customizePrefixCls);
    return <InternalRadio prefixCls={prefixCls} {...rest} />;
}

Radio.Group = InternalRadio.Group;

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

Radio.displayName = 'Radio';
