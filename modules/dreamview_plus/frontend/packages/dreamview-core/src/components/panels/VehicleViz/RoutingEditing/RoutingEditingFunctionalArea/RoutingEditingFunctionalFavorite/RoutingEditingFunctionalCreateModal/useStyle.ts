import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'routing-modal': {
            '& .dreamview-modal-wrap': {
                zIndex: '1040',
            },
            '& .dreamview-modal-mask': {
                zIndex: '1040',
            },
            '& .dreamview-modal-content': {
                width: '492px',
                '& .dreamview-modal-body': {
                    marginTop: '30px',
                    label: {
                        color: '#FFFFFF',
                        fontFamily: 'PingFangSC-Regular',
                        fontWeight: '400',
                    },
                    '& .dreamview-form': {
                        '& .ant-form-item': {
                            marginBottom: '16px',
                        },
                        '& > div:nth-of-type(1)': {
                            '& .ant-form-item-label': {
                                '& label': {
                                    position: 'relative',
                                    top: '4px',
                                },
                            },
                        },
                    },
                },
            },
            '& .dreamview-modal-footer': {
                display: 'flex',
                justifyContent: 'center',
                alignItems: 'center',
                '& > button': {
                    width: '74px',
                    height: '40px',
                    borderRadius: '8px',
                },
                '& > button:nth-of-type(1)': {
                    color: '#FFFFFF',
                    background: '#282B36',
                    border: '1px solid rgba(124,136,153,1)',
                },
                '& > button:nth-of-type(2)': {
                    background: '#3288FA',
                    borderRadius: '8px',
                    marginLeft: '24px !important',
                },
            },
        },

        'routing-form-initial': {
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            fontWeight: '400',
            color: '#FFFFFF',
            marginLeft: '39px',
            marginBottom: '16px',
            display: 'flex',
        },
        'routing-form-initial-content': {
            width: '320px',
            color: '#FFFFFF',
            display: 'flex',
            justifyContent: 'space-between',
        },
        'routing-form-initial-content-heading': {
            width: '111px',
        },
        'routing-form-way': {
            height: '264px',
            border: '1px solid rgba(56,59,69,1)',
            borderRadius: '8px',
            padding: '16px 0px 16px 45px',
            marginBottom: '12px',
        },
        'routing-form-way-con': {
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            fontWeight: '400',
            color: '#FFFFFF',
            display: 'flex',
        },
        'routing-form-way-content': {
            flex: '1',
        },
        'routing-form-way-item': {
            color: '#FFFFFF',
            marginBottom: '8px',
            display: 'flex',
            justifyContent: 'space-between',
        },
        'routing-form-way-item-heading': {
            width: '111px',
        },
        'routing-form-colon': {
            color: '#A6B5CC',
            marginRight: '6px',
        },
        'routing-form-colon-distance': {
            marginLeft: '2px',
        },
        // 'routing-form-loop-times': {
        //     width: '96px',
        //     '& .dreamview-input-number': {
        //         borderColor: 'transparent',
        //         boxShadow: 'none',
        //     },
        //     '& .dreamview-input-number-focused': {
        //         border: '1px solid rgba(50,136,250,1)',
        //     },
        //     '& .dreamview-input-number-input-wrap': {
        //         '& input': {
        //             caretColor: 'none',
        //         },
        //     },
        //     '& .dreamview-input-number-handler-wrap': {
        //         display: 'none',
        //     },
        // },
        'routing-form-loop-disable': {
            background: 'rgb(40, 93, 164)',
            '& .dreamview-switch-handle': {
                background: 'rgb(190, 206, 227)',
                borderRadius: '3px',
            },
        },

        'create-modal-form': {
            '& .ant-form-item-label': {
                '& label': {
                    color: '#A6B5CC !important',
                },
            },
        },
    }));
    return hoc();
}
