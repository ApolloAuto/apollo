import { makeStylesWithProps } from '@dreamview/dreamview-theme';
import { useImagePrak } from '@dreamview/dreamview-ui';

const useHoc = makeStylesWithProps<{ welcomeGuideLogo: string }>()((theme, props) => {
    const tokens = theme.components.setupPage;
    return {
        'welcome-guide-container': {
            width: '100%',
            height: '100%',
            display: 'flex',
            justifyContent: 'center',
            padding: '0px 80px 80px 80px',
            backgroundSize: 'cover',
            backgroundRepeat: 'no-repeat',
            backgroundImage: theme.components.setupPage.backgroundImage,
            backgroundColor: tokens.backgroundColor,
            '& .dreamview-btn-primary': {
                background: tokens.buttonBgColor,
                '&:hover': {
                    background: tokens.buttonBgHoverColor,
                },
                '&:active': {
                    background: tokens.buttonBgActiveColor,
                },
            },
        },
        'welcome-guide-content': {
            width: '100%',
            height: '100%',
            position: 'relative',
        },
        'welcome-guide-head': {
            width: '100%',
            height: '172px',
            display: 'flex',
            justifyContent: 'space-between',
        },
        'welcome-guide-head-text': {
            width: '700px',
            minWidth: '700px',
            paddingTop: '80px',
            paddingBottom: '16px',
        },
        'welcome-guide-head-text-name': {
            height: '50px',
            fontFamily: 'PingFangSC-Semibold',
            fontSize: '36px',
            color: tokens.headNameColor,
            fontWeight: 600,
            backgroundSize: '124px 5px',
            backgroundRepeat: 'no-repeat',
            backgroundPosition: 'left top 35px',
        },
        'welcome-guide-head-text-name-no-login': {
            color: tokens.hadeNameNoLoginColor,
            cursor: 'pointer',
        },
        'welcome-guide-head-text-desc': {
            color: tokens.fontColor,
            ...theme.tokens.typography.content,
        },
        'welcome-guide-head-logo': {
            width: '400px',
            marginTop: '30px',
            backgroundImage: `url(${props.welcomeGuideLogo})`,
            backgroundRepeat: 'no-repeat',
            backgroundSize: '100%',
            backgroundPosition: 'top right',
        },
        policy: {
            ...theme.tokens.typography.content,
            marginBottom: theme.tokens.margin.speace,
        },
        blue: {
            color: theme.tokens.colors.brand3,
            cursor: 'pointer',
            '&:hover': {
                textDecoration: 'underline',
            },
        },
        'welcome-guide-content-tab': {
            minHeight: '390px',
            display: 'flex',
            background: tokens.tabBgColor,
            borderRadius: '0px 0px 12px 12px',
            position: 'absolute',
            top: '55px',
            left: '0px',
            right: '0px',
            bottom: '0px',
            padding: '32px 30px 40px 32px',
        },
        'welcome-guide-content-tab-image': {
            flex: 1,
            borderRadius: '6px',
            backgroundSize: 'cover',
            backgroundRepeat: 'no-repeat',
            backgroundPositionY: '-2px',
        },
        'welcome-guide-content-tab-text': {
            width: '354px',
            marginLeft: '32px',
            position: 'relative',
        },
        'welcome-guide-content-tab-text-desc': {
            fontFamily: 'PingFangSC-Medium',
            fontSize: '14px',
            color: tokens.fontColor,
            fontWeight: '500',
        },
        'welcome-guide-content-tab-text-modules-panel': {
            marginTop: '24px',
            fontFamily: 'PingFangSC-Medium',
            fontSize: '14px',
            color: tokens.fontColor,
            fontWeight: '500',
        },
        'welcome-guide-content-tab-text-modules-panel-desc': {
            marginTop: '2px',
            fontFamily: 'PingFangSC-Regular',
            fontSize: '14px',
            color: '#808B9D',
            fontWeight: 400,
        },
        'welcome-guide-tabs': {
            minWidth: '418px',
            position: 'absolute',
            top: '172px',
            left: '0px',
            right: '0px',
            bottom: '0px',
            '& .dreamview-tabs': {
                '& .dreamview-tabs-tab': {
                    background: tokens.tabBgColor,
                },
                '& .dreamview-tabs-nav-wrap': {
                    '& .dreamview-tabs-tab-active': {
                        background: tokens.tabActiveBgColor,
                        '& .dreamview-tabs-tab-btn': {
                            color: tokens.tabActiveColor,
                            textShadow: 'none',
                            fontFamily: 'PingFangSC-Semibold',
                        },
                    },
                },
            },
            '& .dreamview-tabs-nav': {
                borderRadius: '12px 12px 0px 0px',
                borderBottom: tokens.tabBorder,
                background: tokens.tabBgColor,
                width: '100%',
                height: '56px',
                margin: '0px',
                zIndex: 5,
                '& .dreamview-tabs-nav-wrap': {
                    display: 'flex',
                    justifyContent: 'center',
                    '& .dreamview-tabs-nav-list': {
                        // 导航栏宽度
                        borderRadius: '0px',
                        backgroundColor: 'transparent',
                        '& .dreamview-tabs-tab': {
                            // 导航项宽度
                            width: '140px',
                            fontFamily: 'PingFangSC-Regular',
                            fontSize: '16px',
                            fontWeight: '400',
                            margin: '0px 22px !important',
                            color: tokens.tabColor,
                            '& .dreamview-tabs-tab-btn': {
                                width: '100%',
                                display: 'flex',
                                justifyContent: 'center',
                            },
                        },
                        '& > div:nth-of-type(1)': {
                            display: 'flex',
                            justifyContent: 'right',
                        },
                        '& .dreamview-tabs-tab-active': {
                            fontWeight: '600',
                            fontFamily: 'PingFangSC-Semibold',
                        },
                        '& .dreamview-tabs-ink-bar': {
                            position: 'absolute',
                            display: 'block',
                        },
                    },
                },
            },
            '& .dreamview-tabs-content': {
                position: 'static',
            },
        },
        'enter-this-mode': {
            position: 'absolute',
            left: '0px',
            bottom: '0px',
        },
        'enter-this-mode-btn': {
            width: '204px',
            height: '40px',
            color: 'FFFFFF',
            borderRadius: '6px',
            fontSize: '14px',
            fontWeight: '400',
            fontFamily: 'PingFangSC-Regular',
            '&.dreamview-btn-disabled': {
                background: theme.tokens.colors.divider2,
                color: 'rgba(255,255,255,0.7)',
            },
        },
        'welcome-guide-login-content-text': {
            ...theme.tokens.typography.content,
            fontSize: '16px',
            color: tokens.fontColor,
            margin: '16px 0px 10px 0px',
        },
        'welcome-guide-login-content-image': {
            width: '100%',
            height: '357px',
            borderRadius: '6px',
            backgroundSize: 'cover',
        },
    };
});

export default function useStyle() {
    const welcomeGuideLogo = useImagePrak('welcome_guide_logov2');
    return useHoc({ welcomeGuideLogo });
}
