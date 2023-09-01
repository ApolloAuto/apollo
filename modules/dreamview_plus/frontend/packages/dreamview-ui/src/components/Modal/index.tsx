import React, { useState } from 'react';
import PropTypes from 'prop-types';
import { Modal as InternalModal, ModalProps } from 'antd';
import './index.less';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import { IconIcClose } from '../../icons';

export function Modal(props: ModalProps) {
    const { prefixCls: customizePrefixCls, children, ...rest } = props;
    const prefixCls = getPrefixCls('modal', customizePrefixCls);

    return (
        <InternalModal prefixCls={prefixCls} closeIcon={<IconIcClose />} {...rest}>
            {children}
        </InternalModal>
    );
}

Modal.propTypes = {
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

Modal.defaultProps = {
    open: false,
};

Modal.displayName = 'Modal';
