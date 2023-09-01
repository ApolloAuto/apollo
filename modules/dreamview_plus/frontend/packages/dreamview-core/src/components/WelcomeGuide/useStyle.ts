import { useMakeStyle } from '@dreamview/dreamview-theme';
import welcomeGuideBackground from '@dreamview/dreamview-core/src/assets/welcome_guide_background.png';
import welcomeGuideLogo from '@dreamview/dreamview-core/src/assets/welcome_guide_logov2.png';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'welcome-guide-container': {
            width: '100%',
            height: '100%',
            display: 'flex',
            justifyContent: 'center',
            padding: '0px 80px 80px 80px',
            backgroundSize: 'cover',
            backgroundRepeat: 'no-repeat',
            backgroundImage: `url(${welcomeGuideBackground})`,
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
            color: '#FFFFFF',
            fontWeight: 600,
            backgroundSize: '124px 5px',
            backgroundRepeat: 'no-repeat',
            backgroundPosition: 'left top 35px',
        },
        'welcome-guide-head-text-name-no-login': {
            color: '#3288FA',
            cursor: 'pointer',
        },
        'welcome-guide-head-text-desc': {
            color: theme.tokens.colors.fontColor2,
            ...theme.tokens.typography.content,
        },
        'welcome-guide-head-logo': {
            width: '400px',
            marginTop: '30px',
            backgroundImage: `url(${welcomeGuideLogo})`,
            backgroundRepeat: 'no-repeat',
            backgroundSize: '100%',
            backgroundPosition: 'top right',
        },
        policy: {
            ...theme.tokens.typography.content,
            marginBottom: theme.tokens.margin.speace,
        },
        blue: {
            color: theme.tokens.colors.brand2,
            cursor: 'pointer',
            '&:hover': {
                textDecoration: 'underline',
            },
        },
        'welcome-guide-content-tab': {
            minHeight: '390px',
            display: 'flex',
            background: '#282B36',
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
            backgroundSize: 'cover',
            backgroundRepeat: 'no-repeat',
        },
        'welcome-guide-content-tab-text': {
            width: '354px',
            marginTop: '10px',
            marginLeft: '32px',
            position: 'relative',
        },
        'welcome-guide-content-tab-text-desc': {
            fontFamily: 'PingFangSC-Medium',
            fontSize: '14px',
            color: '#A6B5CC',
            fontWeight: '500',
        },
        'welcome-guide-content-tab-text-modules-panel': {
            marginTop: '24px',
            fontFamily: 'PingFangSC-Medium',
            fontSize: '14px',
            color: '#A6B5CC',
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
            '& .dreamview-tabs-nav': {
                borderRadius: '12px 12px 0px 0px',
                borderBottom: '1px solid #383C4D',
                background: '#282B36',
                width: '100%',
                height: '56px',
                margin: '0px',
                zIndex: 5,
                '& .dreamview-tabs-nav-wrap': {
                    borderBottm: '1px solid #383C4D !important',
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
                            color: '#A6B5CC',
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
                            color: '#3385FF',
                            fontWeight: '600',
                            fontFamily: 'PingFangSC-Semibold',
                        },
                        '& .dreamview-tabs-ink-bar': {
                            width: '%100 !important',
                            position: 'absolute',
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
            fontFamily: 'PingFangSC-Semibold',
            fontSize: '16px',
            color: '#A6B5CC',
            fontWeight: 600,
            margin: '16px 0px 10px 0px',
        },
        'welcome-guide-login-content-image': {
            width: '952px',
            height: '357px',
            borderRadius: '6px',
            backgroundSize: 'cover',
        },
    }));
    return hoc();
}
