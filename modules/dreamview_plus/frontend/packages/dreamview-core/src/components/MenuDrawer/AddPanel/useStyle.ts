import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'add-panel-content': {
            marginTop: '6px',
            height: 'calc(100% - 6px - 78px)',
        },
        'add-panel-item': {
            padding: '0 16px 0 24px',
            margin: '6px 0px 6px 0px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'space-between',
            height: '32px',
            ...theme.tokens.typography.content,

            '&:hover': {
                background: 'rgba(115,193,250,0.08)',
            },
        },
        'panel-thumbnail': {
            width: '204px',
            padding: '12px 12px 14px 12px',
            ...theme.util.flex('column'),
            '& .panel-thumbnail-image': {
                width: '180px',
                height: '135px',
                marginBottom: '8px',
                borderRadius: '6px',
                backgroundColor: '#181A1F',
                backgroundImage: 'cover',
                backgroundSize: '180px 135px',
                backgroundRepeat: 'no-repeat',
            },
            '& .panel-thumbnail-title': {
                alignSelf: 'flex-start',
                color: theme.tokens.colors.fontColor2,
                ...theme.tokens.typography.content,
            },
            '& .panel-thumbnail-description': {
                alignSelf: 'flex-start',
                color: theme.tokens.colors.fontColor3,
                ...theme.tokens.typography.sideText,
            },
        },
    }));
    return hoc();
}
