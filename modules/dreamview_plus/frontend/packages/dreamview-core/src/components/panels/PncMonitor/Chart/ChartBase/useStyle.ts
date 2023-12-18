import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'moniter-item-container': {
            backgroundColor: '#282D38',
            color: theme.tokens.colors.fontColor2,
            borderRadius: '6px',
        },
        'moniter-item-title': {
            ...theme.tokens.typography.sideText,
            lineHeight: '32px',
            textAlign: 'center',
            background: '#343C4D',
            borderRadius: '6px 6px 0px 0px',
            marginBottom: theme.tokens.margin.speace2,
            position: 'relative',
        },
        'moniter-item-title-extra': {
            position: 'absolute',
            top: 0,
            bottom: 0,
            right: '16px',
            display: 'flex',
            alignItems: 'center',
        },
        'moniter-item-chart-container': {
            width: '100%',
            height: '300px',
            position: 'relative',
            '&.autoHeight': {
                height: 0,
                paddingBottom: '100%',
            },
        },

        'moniter-item-chart': {
            position: 'absolute',
            left: 0,
            top: 0,
            right: 0,
            bottom: 0,
        },
        'moniter-item-toolbox': {
            margin: `${theme.tokens.margin.speace} ${theme.tokens.margin.speace2} -23px ${theme.tokens.margin.speace2}`,
            display: 'flex',
            justifyContent: 'space-between',
        },
        'moniter-item-yaxis': {
            fontFamily: 'PingFangSC-Regular',
            fontSize: '12px',
            color: '#808B9D',
        },
        'moniter-item-operate': {
            position: 'relative',
            zIndex: 2,
            display: 'flex',
            alignItems: 'center',
            '& > .anticon': {
                cursor: 'pointer',
                display: 'block',
                '&:hover': {
                    color: theme.tokens.font.reactive.mainHover,
                },
                '&:active': {
                    color: theme.tokens.font.reactive.mainActive,
                },
            },
        },
        'refresh-ic': {
            fontSize: theme.tokens.font.size.large,
            marginLeft: theme.tokens.margin.speace,
        },
    }));

    return hoc();
}
