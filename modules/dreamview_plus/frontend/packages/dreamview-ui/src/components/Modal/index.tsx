import React, { useState } from 'react';
import { Modal as InternalModal, ModalProps, ModalFuncProps } from 'antd';
import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import IcModalConfirmWarning from '@dreamview/dreamview-core/src/assets/svg/routeEditing/modal_confirm_warning';
import { getPrefixCls } from '../../tools/prefixCls/prefixCls';
import IconPark from '../../IconPark';

const useStyle = makeStylesWithProps<{ classname: string }>()((theme, props) => {
    const tokens = theme.components.modal;
    return {
        [props.classname]: {
            '&.dreamview-modal-root .dreamview-modal-mask': {
                zIndex: 1040,
            },
            '&.dreamview-modal-root .dreamview-modal-wrap': {
                zIndex: 1040,
            },
            '&.dreamview-modal-root': {
                '& .dreamview-modal': {
                    fontFamily: 'PingFangSC-Medium',
                    fontSize: '16px',
                    fontWeight: 500,
                    color: tokens.contentColor,

                    '& .dreamview-modal-content': {
                        backgroundColor: tokens.backgroundColor,
                        padding: theme.tokens.padding.speace3,
                        '& .dreamview-modal-close': {
                            position: 'absolute',
                            top: '12px',
                        },
                    },

                    '& .dreamview-modal-header': {
                        color: tokens.headColor,
                        backgroundColor: tokens.backgroundColor,
                        borderBottom: `1px solid ${tokens.divider}`,
                        margin: `-${theme.tokens.margin.speace3}`,
                        marginBottom: theme.tokens.margin.speace3,
                        padding: '0 24px',
                        height: '48px',

                        '& .dreamview-modal-title': {
                            color: tokens.headColor,
                            lineHeight: '48px',
                        },
                    },

                    '& .dreamview-modal-close': {
                        color: tokens.closeIconColor,
                    },

                    '& .dreamview-modal-close:hover': {
                        color: tokens.closeIconColor,
                    },

                    '& .dreamview-modal-footer': {
                        margin: theme.tokens.margin.speace3,
                        marginBottom: 0,
                        '& button': {
                            margin: 0,
                            marginInlineStart: '0 !important',
                        },
                        '& button:first-of-type': {
                            marginRight: theme.tokens.margin.speace2,
                        },
                    },
                    '& .ant-btn-background-ghost': {
                        borderColor: tokens.closeBtnBorderColor,
                        color: tokens.closeBtnColor,
                        '&:hover': {
                            borderColor: tokens.closeBtnBorderHoverColor,
                            color: tokens.closeBtnHoverColor,
                        },
                    },
                },

                '& .dreamview-modal-mask': {
                    backgroundColor: 'rgba(0, 0, 0, 0.5)',
                },
            },
            '& .dreamview-modal-confirm': {
                '& .dreamview-modal-content': {
                    width: '400px',
                    background: '#282B36',
                    borderRadius: '10px',
                    padding: '30px 0px 30px 0px',
                    display: 'flex',
                    justifyContent: 'center',
                    '& .dreamview-modal-body': {
                        maxWidth: '352px',
                        display: 'flex',
                        justifyContent: 'center',
                        '& .dreamview-modal-confirm-content': {
                            maxWidth: '324px',
                            marginLeft: '28px',
                            fontFamily: 'PingFang-SC-Medium',
                            fontSize: '16px',
                            color: '#A6B5CC',
                            fontWeight: 500,
                        },
                        '& .dreamview-modal-confirm-body': {
                            display: 'flex',
                            flexWrap: 'nowrap' as any,
                            position: 'relative',
                            '& > svg': {
                                position: 'absolute',
                                top: '4px',
                            },
                        },
                        '& .dreamview-modal-confirm-btns': {
                            marginTop: '24px',
                            display: 'flex',
                            justifyContent: 'center',
                            '& > button': {
                                width: '72px',
                                height: '40px',
                            },
                            '& > button:nth-child(1)': {
                                color: '#FFFFFF',
                                background: '#282B36',
                                border: '1px solid rgba(124,136,153,1)',
                            },
                            '& > button:nth-child(1):hover': {
                                color: '#3288FA',
                                border: '1px solid #3288FA',
                            },
                            '& > button:nth-child(1):active': {
                                color: '#1252C0',
                                border: '1px solid #1252C0',
                            },
                            '& > button:nth-child(2)': {
                                padding: '4px 12px 4px 12px !important',
                            },
                        },
                    },
                },
            },
        },
    };
});

export function Modal(props: ModalProps) {
    const { prefixCls: customizePrefixCls, children, rootClassName, ...rest } = props;
    const prefixCls = getPrefixCls('modal', customizePrefixCls);
    const { classes, cx } = useStyle({ classname: prefixCls });
    return (
        <InternalModal
            rootClassName={cx(classes[prefixCls], rootClassName)}
            prefixCls={prefixCls}
            closeIcon={<IconPark name='IcClose' />}
            {...rest}
        >
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
