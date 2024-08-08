import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dv-default-guide-skip': {
        ...theme.tokens.typography.content,
        color: theme.tokens.font.color.main,
    },
    'dv-default-guide-current-value': {
        ...theme.tokens.typography.content,
        fontSize: '18px',
        color: theme.components.setupPage.guideStepColor,
    },
    'dv-default-guide-all-value': {
        ...theme.tokens.typography.content,
        color: theme.components.setupPage.guideStepTotalColor,
    },

    'dv-default-guide-back-node': {
        width: '72px',
        height: '40px',
        ...theme.tokens.typography.content,
        color: theme.components.setupPage.guideBackColor,
        backgroundColor: theme.components.setupPage.guideBackBgColor,
        border: theme.components.setupPage.guideBackBorderColor,
        borderRadius: '8px',
        '&:hover': {
            color: '#3288FA',
            border: '1px solid #3288FA',
            backgroundColor: 'transparent',
        },
    },

    'dv-default-guide-next-node': {
        width: '72px',
        height: '40px',
        ...theme.tokens.typography.content,
        color: '#FFFFFF',
        backgroundColor: theme.tokens.colors.brand2,
        borderRadius: '8px',
    },
    'dv-default-guide-close-node': {
        width: '72px',
        height: '40px',
        ...theme.tokens.typography.content,
        color: '#FFFFFF',
        backgroundColor: theme.tokens.colors.brand2,
        borderRadius: '8px',
    },
}));
