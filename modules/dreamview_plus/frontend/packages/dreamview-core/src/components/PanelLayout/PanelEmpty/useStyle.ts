import { useMakeStyle } from '@dreamview/dreamview-theme';

export default function useStyle() {
    const hoc = useMakeStyle((theme) => ({
        'panel-item-add': {
            position: 'absolute',
            left: 0,
            top: 0,
            right: 0,
            bottom: 0,
            display: 'none',
            alignItems: 'center',
            flexDirection: 'column',
            justifyContent: 'center',
            background: 'rgba(15, 16, 20, 0.7)',
            borderRadius: '8px',
            color: theme.tokens.colors.fontColor1,
            fontSize: theme.tokens.font.size.regular,
            '& .anticon': {
                fontSize: '22.5px',
                marginBottom: theme.tokens.margin.speace,
            },
        },
        'panel-item-title': {
            ...theme.tokens.typography.content,
            ...theme.util.textEllipsis,
            color: theme.tokens.colors.fontColor2,
            marginBottom: '2px',
        },
        'panel-item-desc': {
            ...theme.tokens.typography.sideText,
            ...theme.util.textEllipsis2,
            color: theme.tokens.colors.fontColor3,
        },
        'panel-item-img': {
            width: '100%',
            paddingBottom: '75%',
            background: '#181A1F',
            marginBottom: '6px',
            backgroundSize: '100%',
        },
        'panel-empty-container': {
            width: '100%',
            height: '100%',
            background: theme.tokens.colors.background3,
            display: 'flex',
            flexDirection: 'column',
        },
        'panel-item': {
            cursor: 'pointer',
            position: 'relative',
            padding: '12px',
            background: '#282B36',
            border: '1px solid rgba(46,49,60,1)',
            borderRadius: '8px',
            marginBottom: theme.tokens.margin.speace,
            '&:hover .panel-item-add-hover': {
                display: 'flex',
            },
        },
        'panel-empty-title': {
            lineHeight: '78px',
            padding: `0 ${theme.tokens.margin.speace3}`,
            fontFamily: 'PingFangSC-Semibold',
            fontSize: '20px',
            fontWeight: 600,
            color: theme.tokens.colors.fontColor2,
        },
        'panel-list-container': {
            flex: 1,
        },
        'panel-list': {
            padding: `0 ${theme.tokens.margin.speace3}`,
            display: 'flex',
            flexWrap: 'wrap',
            justifyContent: 'space-between',
        },
    }));
    return hoc();
}
