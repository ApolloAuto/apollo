import React from 'react';
import { Tabs as InternalTabs, TabsProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Tabs(props: TabsProps) {
    const { children, className: customClassNames, ...rest } = props;
    const prefixCls = getPrefixCls('tabs', customClassNames);
    return (
        <InternalTabs prefixCls={prefixCls} {...rest}>
            {children}
        </InternalTabs>
    );
}

Tabs.propTypes = {
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

Tabs.defaultProps = {};

Tabs.displayName = 'Tabs';
