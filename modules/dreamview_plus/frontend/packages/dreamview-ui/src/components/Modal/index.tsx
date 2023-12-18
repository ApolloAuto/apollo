import React, { useState } from 'react';
import { Modal as InternalModal, ModalProps, ModalFuncProps } from 'antd';
import IcModalConfirmWarning from '@dreamview/dreamview-core/src/assets/svg/routeEditing/modal_confirm_warning';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import { IconIcClose } from '../../icons';
import './index.less';

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

Modal.confirm = (props: ModalFuncProps) => {
    InternalModal.confirm({
        icon: <IcModalConfirmWarning />,
        autoFocusButton: null,
        ...props,
        className: `${props.className || ''} dreamview-modal-confirm`,
    });
};
