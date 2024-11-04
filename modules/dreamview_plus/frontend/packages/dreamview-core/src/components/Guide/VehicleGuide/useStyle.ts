import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'dv-vehicle-guide-skip': {
        ...theme.tokens.typography.content,
        color: theme.tokens.font.color.main,
    },
    'dv-vehicle-guide-current-value': {
        ...theme.tokens.typography.content,
        fontSize: '18px',
        color: theme.components.setupPage.guideStepColor,
    },
    'dv-vehicle-guide-all-value': {
        ...theme.tokens.typography.content,
        color: theme.components.setupPage.guideStepTotalColor,
    },

    'dv-vehicle-guide-back-node': {
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

    'dv-vehicle-guide-next-node': {
        width: '72px',
        height: '40px',
        ...theme.tokens.typography.content,
        color: '#FFFFFF',
        backgroundColor: theme.tokens.colors.brand2,
        borderRadius: '8px',
    },
    'dv-vehicle-guide-close-node': {
        width: '72px',
        height: '40px',
        ...theme.tokens.typography.content,
        color: '#FFFFFF',
        backgroundColor: theme.tokens.colors.brand2,
        borderRadius: '8px',
    },
}));