import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        disabled: {
            color: '#40454D',
            '& .progress-pointer': {
                display: 'none',
            },
        },
        'player-controlbar-container': {
            height: '100%',
            display: 'flex',
            alignItems: 'center',
            padding: `0 ${theme.tokens.padding.speace3}`,
            color: theme.tokens.colors.fontColor3,
            '& .player-progress-text': {
                marginRight: theme.tokens.margin.speace3,
                fontSize: theme.tokens.font.size.regular,
                width: 110,
            },
            '& .ic-play-btn': {
                cursor: 'pointer',
                fontSize: '24px !important',
                marginRight: '6px',
            },
            '& .player-progress': {
                flex: 1,
                marginRight: theme.tokens.margin.speace2,
            },
        },
    }));

    return hoc();
}
