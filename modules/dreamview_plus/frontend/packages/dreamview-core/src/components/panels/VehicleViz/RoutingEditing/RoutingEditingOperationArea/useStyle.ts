import { makeStyles } from '@dreamview/dreamview-theme';

export default makeStyles((theme) => ({
    'routing-editing-op-con': {
        '& > button:nth-of-type(1)': {
            width: '72px',
            height: '32px',
            marginRight: '16px',
            backgroundColor: theme.components.routingEditing.backgroundColor,
            border: theme.components.routingEditing.border,
            color: theme.components.routingEditing.color,
            '&:hover': {
                color: theme.components.routingEditing.hoverColor,
                backgroundColor: theme.components.routingEditing.backgroundHoverColor,
                border: theme.components.routingEditing.borderHover,
            },
            '&:active': {
                color: theme.components.routingEditing.activeColor,
                backgroundColor: theme.components.routingEditing.backgroundActiveColor,
                border: theme.components.routingEditing.borderActive,
            },
        },
        '& > button:nth-of-type(2)': {
            width: '114px',
            height: '32px',
        },
    },
}));
