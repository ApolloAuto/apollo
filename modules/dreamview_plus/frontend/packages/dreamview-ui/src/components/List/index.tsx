import React from 'react';
import { List as InternalList, ListProps } from 'antd';
// import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function List<T>(props: ListProps<T>) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('list', customizePrefixCls);

    return (
        <InternalList prefixCls={prefixCls} {...rest}>
            {children}
        </InternalList>
    );
}

List.propTypes = {
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

List.defaultProps = {};

List.displayName = 'List';
