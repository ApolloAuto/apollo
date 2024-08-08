import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    disabled: {
        color: '#40454D',
        '& .progress-pointer': {
            display: 'none',
        },
        '& .ic-routing-btn': {
            cursor: 'not-allowed',
        },
    },
    'player-controlbar-container': {
        height: '100%',
        display: 'flex',
        alignItems: 'center',
        padding: `0 ${theme.tokens.padding.speace3}`,
        color: theme.tokens.colors.fontColor4,
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
        '& .ic-routing-btn': {
            cursor: 'pointer',
            fontSize: '18px !important',
            marginRight: '6px',
        },
        '& .player-progress': {
            flex: 1,
            marginRight: theme.tokens.margin.speace2,
        },
    },
}));
