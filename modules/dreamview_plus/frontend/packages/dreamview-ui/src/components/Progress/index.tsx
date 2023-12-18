import React from 'react';
import { Progress as InternalProgress, ProgressProps } from 'antd';
// import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Progress(props: ProgressProps) {
    const { prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('progress', customizePrefixCls);

    return <InternalProgress prefixCls={prefixCls} trailColor='#343947' {...rest} />;
}

// Progress.propTypes = {
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

// Progress.defaultProps = {
//     backgroundColor: null,
//     primary: true,
//     size: 'medium',
//     onClick: undefined,
// };

Progress.displayName = 'Progress';
