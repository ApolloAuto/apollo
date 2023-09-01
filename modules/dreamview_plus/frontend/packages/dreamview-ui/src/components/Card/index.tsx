import React from 'react';
import PropTypes from 'prop-types';
import { Card as InternalCard, CardProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';

export function Card(props: CardProps) {
    const { children, prefixCls: customizePrefixCls, ...rest } = props;
    const prefixCls = getPrefixCls('card', customizePrefixCls);

    return (
        <InternalCard prefixCls={prefixCls} {...rest}>
            {children}
        </InternalCard>
    );
}

Card.propTypes = {
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

Card.defaultProps = {};

Card.displayName = 'Card';
