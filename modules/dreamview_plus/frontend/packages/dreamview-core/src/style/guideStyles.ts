export default (tokens: any) => ({
    '.__floater__open': {},
    '.react-joyride__spotlight': {
        border: '1.5px dashed #76AEFA',
        borderRadius: '12px !important',
        padding: '6px !important',
        background: '#1A1D24',
        display: 'content-box',
        backgroundClip: 'content-box !important',
    },
    '.react-joyride__tooltip': {
        backgroundColor: `${tokens.components.setupPage.guideBgColor} !important`,
        '& h4': {
            color: tokens.components.setupPage.guideTitleColor,
            borderBottom: tokens.components.setupPage.border,
        },
        '& > div > div': {
            color: tokens.components.setupPage.guideColor,
        },
        '& > div:nth-of-type(2)': {
            '& > button': {
                outline: 'none',
                backgroundColor: 'transparent !important',
                padding: '0px !important',
                borderRadius: '0px !important',
                '& > button': {
                    marginLeft: '19px',
                    boxShadow: '0px 0px 0px transparent !important',
                },
            },

            '& > div': {
                '& > button': {
                    padding: '0px !important',
                    paddingTop: '12px !important',
                },
            },
        },
    },
});
