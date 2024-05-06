import React from 'react';
import PropTypes from 'prop-types';
import { Tree as InternalTree, TreeProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Tree(props: TreeProps) {
    const { children, className: customClassNames, ...rest } = props;
    const prefixCls = getPrefixCls('tree', customClassNames);
    return (
        <InternalTree prefixCls={prefixCls} {...rest}>
            {children}
        </InternalTree>
    );
}

Tree.propTypes = {
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

Tree.defaultProps = {};

Tree.displayName = 'Tree';
